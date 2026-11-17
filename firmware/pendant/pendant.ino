// Thoughtaker pendant: records 16 kHz audio to microSD as 60 s IMA ADPCM chunks and serves them
// to the phone over BLE. docs/protocol.md is the contract; section numbers below refer to it.
// Tasks: mic (I2S -> RAM ring), writer (ring -> DC removal, VAD, ADPCM -> SD), ble (requests and events).
#include <ESP_I2S.h>
#include <SD.h>
#include <vfs_api.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <atomic>
#include <unistd.h>
#include "esp_random.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "ff.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "chunk.h"

const uint32_t RING_SAMPLES = 32768;  // 2.05 s; must be a power of two (index masking)
const uint32_t BATCH_BLOCKS = 8;      // SD writes of 2 KB, whole blocks only
const uint32_t FLUSH_MS = 2000;
const uint64_t MIN_FREE_BYTES = 2ull * (HEADER_BYTES + CHUNK_BLOCKS * BLOCK_BYTES);
const uint32_t OPEN_RETRY_MS = 5000;  // while the card is full or a chunk failed to write
const uint32_t PAIRING_WINDOW_MS = 120000;
const uint32_t PAIRING_HOLD_MS = 3000;  // hold BOOT this long to forget the phone and pair a new one
// Drop a link that has not encrypted with a bond so an idle peer cannot hold the only connection.
// A bonded phone encrypts on its first RX write, well within this.
const uint32_t UNTRUSTED_LINK_MS = 5000;
const uint32_t PAIRING_LINK_MS = 30000;  // in the pairing window the user may still be accepting the phone's pairing prompt
const uint16_t MIN_MTU = 247;
const int NOTIFY_MIN_FREE_MBUFS = 8;  // a 507 B frame takes ~2 of the 320 B blocks; margin for the BLE host's own use
const size_t MAX_FRAME = 512;  // ATT caps an attribute value at 512 bytes even when MTU - 3 is larger
const size_t MAX_LIST_ENTRIES = (MAX_FRAME - 9) / 9;
const uint64_t MIN_EPOCH_MS = 1577836800000ull;  // 2020-01-01; anything earlier is a phone bug

// Energy VAD (per 505-sample block, after DC removal). First guesses: calibrate on the bench.
const uint32_t VAD_MIN_ENERGY = 60 * 60;  // mean square; below this is silence whatever the floor says
const uint32_t VAD_RATIO = 8;             // ~9 dB over the noise floor
const uint8_t VAD_MIN_RUN = 3;            // ~95 ms of consecutive loud blocks

const char *SERVICE_UUID = "3f66b6a2-8fd5-423c-9ef1-494e6f3b9100";
const char *RX_UUID = "3f66b6a2-8fd5-423c-9ef1-494e6f3b9101";
const char *TX_UUID = "3f66b6a2-8fd5-423c-9ef1-494e6f3b9102";

enum : uint8_t { OP_GET_STATUS = 1, OP_SET_TIME, OP_LIST, OP_PULL, OP_ACK, OP_SET_MUTE, OP_SET_WAKE_WORD };
enum : uint8_t { ST_OK, ERR_UNKNOWN_OP, ERR_BAD_LENGTH, ERR_BAD_VALUE, ERR_NOT_FOUND, ERR_RANGE, ERR_CRC, ERR_IO, ERR_UNSUPPORTED, ERR_MTU };
enum : uint8_t { FRAME_RESP = 1, FRAME_DATA, FRAME_EVENT };
enum : uint8_t { EVENT_CHUNK_READY = 1, EVENT_BOOKMARK };

// fs::SDFS keeps its FatFs drive number protected; LIST reads file sizes straight from directory entries with it.
struct Card : fs::SDFS {
  Card() : SDFS(FSImplPtr(new VFSImpl())) {}
  uint8_t drive() const { return _pdrv; }
} card;

Preferences prefs;
I2SClass i2s;

uint32_t storeId, bootId, nextSeq = 1;
bool cardReady;  // card mounted and /TK readable at boot; sdError on its own means only the last chunk write failed
std::atomic<bool> muted{false}, timeSynced{false}, storageFull{false}, sdError{false};
std::atomic<bool> headerDirty{false}, bleConnected{false}, linkTrusted{false};
std::atomic<int64_t> epochOffsetMs{0};
std::atomic<uint32_t> droppedSamples{0}, chunksWritten{0}, pendingChunks{0}, worstWriteUs{0}, ledOffAtMs{0};

// Single-producer (mic) single-consumer (writer) ring. Counters run free; the index is counter % size.
int16_t ring[RING_SAMPLES];
std::atomic<uint32_t> ringHead{0}, ringTail{0};
portMUX_TYPE headMux = portMUX_INITIALIZER_UNLOCKED;
int64_t headUs = 0;  // esp_timer time when the newest ring sample arrived; guarded by headMux with ringHead

struct Request {
  uint16_t len;
  uint8_t data[16];  // longest valid request is PULL, 14 bytes
};
struct Event {
  uint8_t len;
  uint8_t data[18];
};
QueueHandle_t requests, events, bookmarks;
TaskHandle_t bleTaskHandle;
BLECharacteristic *tx;
std::atomic<uint16_t> connHandle{0};
std::atomic<int> notifyStatus{-1};
std::atomic<bool> pairingOpen{false};
std::atomic<uint32_t> pairingOpenedMs{0};
uint32_t connectedAtMs;

uint64_t uptimeMs() { return (uint64_t)esp_timer_get_time() / 1000; }
bool pairingWindowOpen() { return pairingOpen && millis() - pairingOpenedMs < PAIRING_WINDOW_MS; }
void openPairingWindow() {
  pairingOpenedMs = millis();
  pairingOpen = true;
}
bool isRecording() { return !muted && !storageFull && !sdError; }

void chunkPath(char *out, char cls, uint32_t seq) {
  snprintf(out, 20, "/TK/%c%07lX.%s", cls, (unsigned long)seq, cls == 'P' ? "PRT" : "WAV");
}

char classOf(uint16_t flags) { return flags & FLAG_BOOKMARKED ? 'B' : flags & FLAG_SPEECH ? 'S' : 'Q'; }
uint8_t listFlags(char cls) { return cls == 'B' ? FLAG_BOOKMARKED : cls == 'S' ? FLAG_SPEECH : 0; }

bool parseChunkName(const char *name, char &cls, uint32_t &seq) {
  if (strlen(name) != 12 || name[8] != '.' || !strchr("PBSQ", name[0])) return false;
  if (strcmp(name + 9, name[0] == 'P' ? "PRT" : "WAV")) return false;
  for (int i = 1; i < 8; i++)
    if (!isxdigit((unsigned char)name[i]) || islower((unsigned char)name[i])) return false;
  cls = name[0];
  seq = strtoul(name + 1, nullptr, 16);
  return true;
}

// Calls fn(cls, seq, size) for every chunk file. Uses FatFs directly: the Arduino FS API re-scans the
// directory to get each file's size, which is quadratic in the number of chunks.
template <typename Fn> bool forEachChunk(Fn fn) {
  char dirPath[8];
  snprintf(dirPath, sizeof(dirPath), "%u:/TK", card.drive());
  FF_DIR dir;
  FILINFO info;
  if (f_opendir(&dir, dirPath) != FR_OK) return false;
  FRESULT res;
  while ((res = f_readdir(&dir, &info)) == FR_OK && info.fname[0]) {
    char cls;
    uint32_t seq;
    if (!(info.fattrib & AM_DIR) && parseChunkName(info.fname, cls, seq)) fn(cls, seq, (uint32_t)info.fsize);
  }
  f_closedir(&dir);
  return res == FR_OK;
}

uint64_t freeBytes() { return card.totalBytes() - card.usedBytes(); }

void noteWriteTime(uint32_t us) {
  uint32_t worst = worstWriteUs;
  while (us > worst && !worstWriteUs.compare_exchange_weak(worst, us)) {}
}

void queueEvent(const uint8_t *data, uint8_t len) {
  if (!bleConnected) return;  // the phone LISTs after connecting, so nothing is lost
  Event e;
  e.len = len;
  memcpy(e.data, data, len);
  xQueueSend(events, &e, 0);
}

// The one bookmark entry point: the serial 'b' command calls it today, the wake-word detector will later.
// Safe from any task; the writer attaches it to the chunk that holds this moment.
void bookmark() {
  uint64_t t = uptimeMs();
  digitalWrite(LED_BUILTIN, LOW);  // active LOW
  ledOffAtMs = millis() + 300;
  if (xQueueSend(bookmarks, &t, 0) != pdTRUE) Serial.println("bookmark dropped: queue full");
}

// ---------------------------------------------------------------- mic task

bool startMic() {
  i2s.setPins(D3, D4, -1, D5);  // SCK, WS, no output, mic data
  return i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)
         && i2s.configureRX(SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_RX_TRANSFORM_32_TO_16);
}

void micTask(void *) {
  static int16_t buf[320];  // 20 ms
  bool running = false;
  for (;;) {
    if (muted) {
      if (running) i2s.end();
      running = false;
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (!running && !(running = startMic())) {
      Serial.println("I2S init failed, retrying");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    size_t got = i2s.readBytes((char *)buf, sizeof(buf)) / sizeof(int16_t);
    uint32_t head = ringHead, space = RING_SAMPLES - (head - ringTail);
    size_t n = got < space ? got : space;
    droppedSamples += got - n;  // never block on the writer: it may be stuck in a slow SD write
    if (!n) continue;
    for (size_t i = 0; i < n; i++) ring[(head + i) & (RING_SAMPLES - 1)] = buf[i];
    portENTER_CRITICAL(&headMux);
    ringHead = head + n;
    headUs = esp_timer_get_time();
    portEXIT_CRITICAL(&headMux);
  }
}

// ---------------------------------------------------------------- writer task (owns the open chunk)

struct Recording {
  bool open;
  File file;
  ChunkHeader hdr;          // hdr.blocks stays 0 until finalize (1.6 rule 1)
  uint32_t blocks;          // encoded so far, including the batch
  uint32_t diskBlocks;      // written to the file
  uint32_t crc;             // running CRC of the audio written so far (1.5)
  uint32_t batchBlocks;
  uint32_t lastFlushMs;
  uint8_t batch[BATCH_BLOCKS * BLOCK_BYTES];
} rec;

int32_t dcQ8;  // DC estimate in Q8
bool dcSeeded;
uint32_t noiseFloor = UINT32_MAX;
uint8_t loudRun, stepIndex;

void failRecording(const char *why) {
  Serial.printf("SD error (%s): recording stopped, retrying; chunk %lu left for recovery at next boot\n", why, (unsigned long)rec.hdr.seq);
  sdError = true;
  rec.file.close();
  rec.open = false;
}

bool writeHeader(uint32_t blocks) {
  uint8_t h[HEADER_BYTES];
  if (timeSynced) {
    rec.hdr.flags |= FLAG_TIME_SYNCED;
    rec.hdr.epochOffsetMs = epochOffsetMs;
  }
  rec.hdr.blocks = blocks;
  buildHeader(h, rec.hdr);
  if (blocks) put32(h + 92, rec.hdr.crc = esp_rom_crc32_le(rec.crc, h, HEADER_BYTES));  // crc field is 0 in the input
  uint32_t t = micros();
  bool ok = rec.file.seek(0) && rec.file.write(h, HEADER_BYTES) == HEADER_BYTES
            && rec.file.seek(HEADER_BYTES + rec.diskBlocks * BLOCK_BYTES);
  rec.file.flush();
  noteWriteTime(micros() - t);
  return ok;
}

bool writeBatch() {
  size_t n = rec.batchBlocks * BLOCK_BYTES;
  if (!n) return true;
  uint32_t t = micros();
  bool ok = rec.file.write(rec.batch, n) == n;
  noteWriteTime(micros() - t);
  if (!ok) return false;
  rec.crc = esp_rom_crc32_le(rec.crc, rec.batch, n);
  rec.diskBlocks += rec.batchBlocks;
  rec.batchBlocks = 0;
  return true;
}

bool openChunk(int64_t firstSampleUs, bool continuous) {
  if (freeBytes() < MIN_FREE_BYTES) {
    if (!storageFull) Serial.println("storage full: recording paused until chunks are acked");
    storageFull = true;
    return false;
  }
  storageFull = false;
  rec.hdr = {};
  rec.hdr.storeId = storeId;
  rec.hdr.seq = nextSeq++;
  rec.hdr.bootId = bootId;
  rec.hdr.startUptimeMs = (uint64_t)firstSampleUs / 1000;
  prefs.putUInt("next_seq", nextSeq);
  char path[20];
  chunkPath(path, 'P', rec.hdr.seq);
  rec.file = card.open(path, FILE_WRITE);
  rec.open = true;
  rec.blocks = rec.diskBlocks = rec.batchBlocks = rec.crc = 0;
  rec.lastFlushMs = millis();
  if (!continuous) dcSeeded = false;  // the mic restarted, so the old DC estimate is stale
  if (!rec.file || !writeHeader(0)) {
    failRecording("open");
    return false;
  }
  sdError = false;
  return true;
}

void finalizeChunk() {
  if (!writeBatch() || !writeHeader(rec.blocks)) return failRecording("finalize");
  rec.file.close();
  rec.open = false;
  char from[20], to[20];
  char cls = classOf(rec.hdr.flags);
  chunkPath(from, 'P', rec.hdr.seq);
  chunkPath(to, cls, rec.hdr.seq);
  if (!card.rename(from, to)) return failRecording("rename");  // the rename is the commit point (1.6 rule 4)
  chunksWritten++;
  pendingChunks++;
  uint8_t e[11] = {FRAME_EVENT, EVENT_CHUNK_READY};
  put32(e + 2, rec.hdr.seq);
  put32(e + 6, HEADER_BYTES + rec.blocks * BLOCK_BYTES);
  e[10] = listFlags(cls);
  queueEvent(e, sizeof(e));
}

void addBookmark(uint64_t t) {
  uint32_t offset = t > rec.hdr.startUptimeMs ? (uint32_t)(t - rec.hdr.startUptimeMs) : 0;
  if (rec.hdr.bookmarkCount < MAX_BOOKMARKS) rec.hdr.bookmarkMs[rec.hdr.bookmarkCount++] = offset;
  rec.hdr.flags |= FLAG_BOOKMARKED;
  headerDirty = true;
  Serial.printf("bookmark: chunk %lu at %lu ms\n", (unsigned long)rec.hdr.seq, (unsigned long)offset);
  uint8_t e[18] = {FRAME_EVENT, EVENT_BOOKMARK};
  put32(e + 2, rec.hdr.seq);
  put32(e + 6, offset);
  put64(e + 10, t);
  queueEvent(e, sizeof(e));
}

// DC-blocking high-pass, energy VAD, then ADPCM into the batch.
void encodeBlock(int16_t *pcm) {
  uint64_t sumSq = 0;
  for (uint32_t i = 0; i < BLOCK_SAMPLES; i++) {
    int32_t x = pcm[i];
    if (!dcSeeded) {
      dcQ8 = x * 256;
      dcSeeded = true;
    }
    dcQ8 += (x * 256 - dcQ8) >> 10;  // one-pole tracker, ~2.5 Hz corner: follows the INMP441's DC drift
    int32_t y = x - (dcQ8 >> 8);
    y = y < -32768 ? -32768 : y > 32767 ? 32767 : y;
    pcm[i] = (int16_t)y;
    sumSq += (uint64_t)(y * y);
  }
  uint32_t energy = (uint32_t)(sumSq / BLOCK_SAMPLES);
  if (energy < noiseFloor) noiseFloor = energy;
  else noiseFloor += noiseFloor / 512 + 1;  // doubles in ~11 s, so a louder room re-learns its floor
  bool loud = energy > VAD_MIN_ENERGY && energy > (uint64_t)noiseFloor * VAD_RATIO;
  loudRun = loud ? (loudRun < 255 ? loudRun + 1 : 255) : 0;
  if (loudRun >= VAD_MIN_RUN && !(rec.hdr.flags & FLAG_SPEECH)) {
    rec.hdr.flags |= FLAG_SPEECH;
    headerDirty = true;
  }
  adpcmEncodeBlock(pcm, rec.batch + rec.batchBlocks * BLOCK_BYTES, stepIndex);
  rec.batchBlocks++;
  rec.blocks++;
}

void writerTask(void *) {
  static int16_t pcm[BLOCK_SAMPLES];
  uint32_t lastOpenTryMs = 0;
  bool continuous = false;
  for (;;) {
    if (rec.open && headerDirty.exchange(false) && !writeHeader(0)) failRecording("header");
    if (rec.open && millis() - rec.lastFlushMs >= FLUSH_MS) {
      if (!writeBatch()) failRecording("write");
      else {
        uint32_t t = micros();
        rec.file.flush();  // FAT only records the new file size on flush or close
        noteWriteTime(micros() - t);
        rec.lastFlushMs = millis();
      }
    }

    uint32_t tail = ringTail;
    portENTER_CRITICAL(&headMux);
    uint32_t head = ringHead;
    int64_t newestUs = headUs;
    portEXIT_CRITICAL(&headMux);
    if (head - tail < BLOCK_SAMPLES) {
      if (muted) {  // the mic has stopped and the ring is drained: close the chunk, drop the partial block
        if (rec.open) finalizeChunk();
        continuous = false;
        ringTail = head;
      }
      if (!isRecording()) xQueueReset(bookmarks);  // nothing is recording that a bookmark could mark
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (!rec.open) {
      bool canTry = cardReady && (!(storageFull || sdError) || millis() - lastOpenTryMs >= OPEN_RETRY_MS);
      if (canTry) lastOpenTryMs = millis();
      int64_t firstSampleUs = newestUs - (int64_t)(head - tail) * 1000000 / SAMPLE_RATE;
      if (!canTry || !openChunk(firstSampleUs, continuous)) {
        ringTail = tail + BLOCK_SAMPLES;  // not recording: discard
        continuous = false;
        continue;
      }
    }

    for (uint32_t i = 0; i < BLOCK_SAMPLES; i++) pcm[i] = ring[(tail + i) & (RING_SAMPLES - 1)];
    ringTail = tail + BLOCK_SAMPLES;
    encodeBlock(pcm);

    uint64_t blockEndMs = rec.hdr.startUptimeMs + (uint64_t)rec.blocks * BLOCK_SAMPLES * 1000 / SAMPLE_RATE;
    uint64_t t;
    while (xQueuePeek(bookmarks, &t, 0) == pdTRUE && t <= blockEndMs) {
      xQueueReceive(bookmarks, &t, 0);
      addBookmark(t);
    }

    if (rec.batchBlocks == BATCH_BLOCKS && !writeBatch()) {
      failRecording("write");
      continue;
    }
    if (rec.blocks == CHUNK_BLOCKS) {
      finalizeChunk();
      continuous = !sdError;
    }
  }
}

// ---------------------------------------------------------------- storage setup and crash recovery (1.6)

void recoverPartial(uint32_t seq) {
  char path[20];
  chunkPath(path, 'P', seq);
  File f = card.open(path, "r+");
  if (!f) {
    Serial.printf("recovery: cannot open %s, will retry next boot\n", path);
    return;
  }
  uint8_t h[HEADER_BYTES];
  ChunkHeader c;
  size_t len = f.size();
  uint32_t blocks = len > HEADER_BYTES ? (len - HEADER_BYTES) / BLOCK_BYTES : 0;
  if (len < HEADER_BYTES || f.read(h, HEADER_BYTES) != HEADER_BYTES || !parseHeader(h, c) || c.seq != seq || !blocks) {
    f.close();
    Serial.printf("recovery: %s is invalid or empty, deleting\n", path);
    card.remove(path);
    return;
  }
  uint32_t durationMs = (uint32_t)((uint64_t)blocks * BLOCK_SAMPLES * 1000 / SAMPLE_RATE);
  uint8_t kept = 0;
  for (uint8_t i = 0; i < c.bookmarkCount; i++)
    if (c.bookmarkMs[i] < durationMs) c.bookmarkMs[kept++] = c.bookmarkMs[i];
  c.bookmarkCount = kept;
  c.flags = (uint16_t)((c.flags & ~FLAG_BOOKMARKED) | (kept ? FLAG_BOOKMARKED : 0) | FLAG_RECOVERED);
  c.blocks = blocks;
  c.crc = 0;

  static uint8_t buf[4096];
  uint32_t crc = 0;
  for (uint32_t left = blocks * BLOCK_BYTES; left;) {
    size_t n = left < sizeof(buf) ? left : sizeof(buf);
    if (f.read(buf, n) != n) {
      f.close();
      Serial.printf("recovery: read failed on %s, will retry next boot\n", path);
      return;
    }
    crc = esp_rom_crc32_le(crc, buf, n);
    left -= n;
  }
  buildHeader(h, c);
  put32(h + 92, esp_rom_crc32_le(crc, h, HEADER_BYTES));
  bool ok = f.seek(0) && f.write(h, HEADER_BYTES) == HEADER_BYTES;
  f.close();
  // Cut a partial trailing block (rare: writes are whole blocks) so LIST can use the directory size.
  char fullPath[24], to[20];
  snprintf(fullPath, sizeof(fullPath), "/sd%s", path);
  chunkPath(to, classOf(c.flags), seq);
  size_t validLen = HEADER_BYTES + blocks * BLOCK_BYTES;
  if (!ok || (len != validLen && truncate(fullPath, validLen)) || !card.rename(path, to)) {
    Serial.printf("recovery: write failed on %s, will retry next boot\n", path);
    return;
  }
  Serial.printf("recovery: %s -> %s (%lu blocks)\n", path, to, (unsigned long)blocks);
}

void startStorage() {
  if (!card.begin(D2, SPI, 4000000) || card.cardType() == CARD_NONE) {
    Serial.println("SD: no card - recording disabled");
    sdError = true;
    return;
  }
  card.mkdir("/TK");
  uint32_t partial[4], partialCount = 0;  // collect first: renaming while iterating the directory is unsafe
  forEachChunk([&](char cls, uint32_t seq, uint32_t) {
    if (cls == 'P' && partialCount < 4) partial[partialCount++] = seq;
  });
  for (uint32_t i = 0; i < partialCount; i++) recoverPartial(partial[i]);

  uint32_t maxSeq = 0, pending = 0;
  if (!forEachChunk([&](char cls, uint32_t seq, uint32_t) {
        if (seq > maxSeq) maxSeq = seq;
        if (cls != 'P') pending++;
      })) {
    Serial.println("SD: cannot read /TK - recording disabled");
    sdError = true;
    return;
  }
  pendingChunks = pending;
  uint32_t saved = prefs.getUInt("next_seq", 1);
  nextSeq = saved > maxSeq + 1 ? saved : maxSeq + 1;
  // The first free-space query can scan the whole FAT; do it now, before audio is waiting on the card.
  Serial.printf("SD: %llu MB free, %lu chunks pending, next seq %lu\n", freeBytes() / 1048576, (unsigned long)pending, (unsigned long)nextSeq);
  cardReady = true;
}

// ---------------------------------------------------------------- BLE (2, 3)

// The core's GAP handler answers a repeat pairing by deleting the old bond, which would let anyone spoofing the
// owner's address erase it. That handler is private; explicit instantiation is the one legal way to take its address.
template <int (*Fn)(ble_gap_event *, void *)> struct CoreGap {
  friend int coreGapHandler(ble_gap_event *e, void *arg) { return Fn(e, arg); }
};
template struct CoreGap<&BLEServer::handleGATTServerEvent>;
int coreGapHandler(ble_gap_event *e, void *arg);

int gapHandler(ble_gap_event *e, void *arg) {
  if (e->type == BLE_GAP_EVENT_REPEAT_PAIRING && !pairingWindowOpen()) return BLE_GAP_REPEAT_PAIRING_IGNORE;
  return coreGapHandler(e, arg);
}

class ServerCallbacks : public BLEServerCallbacks {
  using BLEServerCallbacks::onConnect;
  using BLEServerCallbacks::onDisconnect;
  void onConnect(BLEServer *server, ble_gap_conn_desc *desc) override {
    ble_gap_set_event_cb(desc->conn_handle, gapHandler, server);
    // Outside the pairing window a new phone may still encrypt, but cannot bond, and is dropped below.
    ble_hs_cfg.sm_bonding = pairingWindowOpen();
    connHandle = desc->conn_handle;
    connectedAtMs = millis();
    linkTrusted = false;
    xQueueReset(requests);
    bleConnected = true;
  }
  void onDisconnect(BLEServer *, ble_gap_conn_desc *) override {
    bleConnected = false;
    linkTrusted = false;
  }
};

class SecurityCallbacks : public BLESecurityCallbacks {
  void onAuthenticationComplete(ble_gap_conn_desc *desc) override {
    if (desc->sec_state.encrypted && desc->sec_state.bonded) {
      linkTrusted = true;
      pairingOpen = false;  // one phone per window, so only one bond is ever kept
      if (!prefs.getBool("paired", false)) prefs.putBool("paired", true);
      return;
    }
    Serial.println("BLE: link not encrypted with a bond, disconnecting");
    ble_gap_terminate(desc->conn_handle, BLE_ERR_AUTH_FAIL);
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  using BLECharacteristicCallbacks::onWrite;
  void onWrite(BLECharacteristic *c) override {
    if (!linkTrusted) return;  // an encrypted but unbonded link is dropped asynchronously; its writes must not run
    Request r;
    size_t n = c->getLength();
    r.len = (uint16_t)(n > 0xFFFF ? 0xFFFF : n);
    memcpy(r.data, c->getData(), n < sizeof(r.data) ? n : sizeof(r.data));
    xQueueSend(requests, &r, 0);
  }
};

class TxCallbacks : public BLECharacteristicCallbacks {
  // notify() reports its own result synchronously in the calling task; the host task's later
  // NOTIFY_TX reports must not overwrite it.
  void onStatus(BLECharacteristic *, Status s, uint32_t) override {
    if (xTaskGetCurrentTaskHandle() == bleTaskHandle) notifyStatus = s;
  }
};

void startBle() {
  BLEDevice::init("Thoughtaker");
  BLEDevice::setMTU(517);
  new BLESecurity();  // its constructor sets key distribution (encryption + identity keys)
  BLESecurity::setCapability(ESP_IO_CAP_NONE);
  BLESecurity::setAuthenticationMode(true, false, true);
  BLEDevice::setSecurityCallbacks(new SecurityCallbacks());
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  server->advertiseOnDisconnect(true);
  BLEService *service = server->createService(SERVICE_UUID);
  BLECharacteristic *rx = service->createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_ENC);
  tx = service->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ_ENC);
  rx->setCallbacks(new RxCallbacks());
  tx->setCallbacks(new TxCallbacks());
  service->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
}

// Retries the same frame while the stack is out of buffers; never skips one (3.6).
bool sendFrame(const uint8_t *data, size_t len) {
  for (int tries = 0; tries < 500 && bleConnected; tries++) {
    // An empty pool makes the core hand NimBLE a NULL mbuf, which ends in an assert (no connection) in the core's read handler.
    if (os_msys_num_free() < NOTIFY_MIN_FREE_MBUFS) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
    notifyStatus = -1;
    tx->setValue(data, len);
    tx->notify();
    if (notifyStatus == BLECharacteristicCallbacks::SUCCESS_NOTIFY) return true;
    if (notifyStatus != BLECharacteristicCallbacks::ERROR_GATT) return false;  // not subscribed or not encrypted
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

void sendEvents() {
  Event e;
  while (xQueueReceive(events, &e, 0) == pdTRUE) sendFrame(e.data, e.len);
}

uint8_t frame[MAX_FRAME];  // BLE task only

void respond(uint8_t op, uint8_t reqId, uint8_t status, size_t payloadLen = 0) {
  frame[0] = FRAME_RESP;
  frame[1] = op;
  frame[2] = reqId;
  frame[3] = status;
  sendFrame(frame, 4 + (status == ST_OK ? payloadLen : 0));
}

uint32_t listKeys[MAX_LIST_ENTRIES], listCount;  // the last LIST response, as class << 28 | seq

// A failed open scans the whole of /TK, so try the class the last LIST reported before guessing.
File openFinalized(uint32_t seq, char *path) {
  char hint = 0;
  for (uint32_t i = 0; i < listCount; i++)
    if ((listKeys[i] & 0x0FFFFFFF) == seq) hint = "BSQ"[listKeys[i] >> 28];
  const char order[] = {hint, 'B', 'S', 'Q'};
  for (int i = 0; i < 4; i++) {
    if (!order[i] || (i && order[i] == hint)) continue;
    chunkPath(path, order[i], seq);
    File f = card.open(path);
    if (f) return f;
  }
  return File();
}

void handleStatus(uint8_t id) {
  uint8_t *s = frame + 4;
  memset(s, 0, 32);
  put16(s, 1);
  s[2] = (uint8_t)(isRecording() | muted << 1 | timeSynced << 3 | storageFull << 4 | sdError << 5 | pairingWindowOpen() << 6);
  s[3] = 0xFF;  // battery unknown until the D0 ADC is wired
  put32(s + 4, storeId);
  put32(s + 8, bootId);
  put64(s + 12, uptimeMs());
  if (cardReady) {
    uint64_t total = card.totalBytes();
    put32(s + 20, (uint32_t)(freeBytes() / 1024));
    put32(s + 24, (uint32_t)(total / 1024));
  }
  put32(s + 28, pendingChunks);
  respond(OP_GET_STATUS, id, ST_OK, 32);
}

void handleList(uint8_t id, uint8_t maxEntries, uint16_t mtu) {
  if (!maxEntries) return respond(OP_LIST, id, ERR_BAD_VALUE);
  if (!cardReady) return respond(OP_LIST, id, ERR_IO);
  size_t frameMax = mtu - 3u < MAX_FRAME ? mtu - 3u : MAX_FRAME;
  uint32_t cap = (uint32_t)((frameMax - 9) / 9);
  if (maxEntries < cap) cap = maxEntries;
  static uint32_t sizes[MAX_LIST_ENTRIES];
  listCount = 0;  // listKeys is rebuilt in place below
  uint32_t total = 0, count = 0;
  bool ok = forEachChunk([&](char cls, uint32_t seq, uint32_t size) {
    if (cls == 'P') return;
    total++;
    uint32_t key = (uint32_t)(cls == 'B' ? 0 : cls == 'S' ? 1 : 2) << 28 | seq;  // seq < 2^28: sync order in one number
    if (count == cap && key >= listKeys[count - 1]) return;
    uint32_t i = count < cap ? count++ : cap - 1;
    for (; i > 0 && listKeys[i - 1] > key; i--) {
      listKeys[i] = listKeys[i - 1];
      sizes[i] = sizes[i - 1];
    }
    listKeys[i] = key;
    sizes[i] = size;
  });
  if (!ok) return respond(OP_LIST, id, ERR_IO);
  listCount = count;
  pendingChunks = total;
  uint8_t *p = frame + 4;
  put32(p, total);
  p[4] = (uint8_t)count;
  for (uint32_t i = 0; i < count; i++) {
    uint8_t *e = p + 5 + 9 * i;
    put32(e, listKeys[i] & 0x0FFFFFFF);
    put32(e + 4, sizes[i]);
    e[8] = listFlags("BSQ"[listKeys[i] >> 28]);
  }
  respond(OP_LIST, id, ST_OK, 5 + 9 * count);
}

void handlePull(uint8_t id, const uint8_t *p, uint16_t mtu) {
  uint32_t seq = get32(p), offset = get32(p + 4), length = get32(p + 8);
  if (!cardReady) return respond(OP_PULL, id, ERR_IO);
  char path[20];
  File f = openFinalized(seq, path);
  if (!f) return respond(OP_PULL, id, ERR_NOT_FOUND);
  uint8_t sizeField[4];
  if (!f.seek(508) || f.read(sizeField, 4) != 4) {
    f.close();
    return respond(OP_PULL, id, ERR_IO);
  }
  uint32_t validLen = HEADER_BYTES + get32(sizeField);
  if (!length || length > 65536 || offset > validLen || length > validLen - offset) {
    f.close();
    return respond(OP_PULL, id, ERR_RANGE);
  }
  size_t frameMax = mtu - 3u < MAX_FRAME ? mtu - 3u : MAX_FRAME;
  uint32_t perFrame = (uint32_t)frameMax - 5;
  static uint8_t buf[8 * (MAX_FRAME - 5)];  // whole frames per SD read
  if (!f.seek(offset)) {
    f.close();
    return respond(OP_PULL, id, ERR_IO);
  }
  for (uint32_t sent = 0; sent < length;) {
    uint32_t want = length - sent < 8 * perFrame ? length - sent : 8 * perFrame;
    if (f.read(buf, want) != want) {
      f.close();
      return respond(OP_PULL, id, ERR_IO);
    }
    for (uint32_t at = 0; at < want; at += perFrame) {
      uint32_t n = want - at < perFrame ? want - at : perFrame;
      frame[0] = FRAME_DATA;
      put32(frame + 1, offset + sent + at);
      memcpy(frame + 5, buf + at, n);
      if (!sendFrame(frame, 5 + n)) {  // link gone: the phone resumes from its last good offset
        f.close();
        return;
      }
    }
    sent += want;
    sendEvents();
  }
  f.close();
  respond(OP_PULL, id, ST_OK);
}

void handleAck(uint8_t id, const uint8_t *p) {
  uint32_t seq = get32(p), crc = get32(p + 4);
  if (!cardReady) return respond(OP_ACK, id, ERR_IO);
  char path[20];
  File f = openFinalized(seq, path);
  if (!f) return respond(OP_ACK, id, ST_OK);  // already gone: a retried ACK must not fail
  uint8_t stored[4];
  bool ok = f.seek(92) && f.read(stored, 4) == 4;
  f.close();
  if (!ok) return respond(OP_ACK, id, ERR_IO);
  if (get32(stored) != crc) return respond(OP_ACK, id, ERR_CRC);
  if (!card.remove(path)) return respond(OP_ACK, id, ERR_IO);
  if (pendingChunks) pendingChunks--;
  respond(OP_ACK, id, ST_OK);
}

void handleRequest(const Request &r) {
  if (r.len < 2) return respond(0, 0, ERR_BAD_LENGTH);
  uint8_t op = r.data[0], id = r.data[1];
  const uint8_t *p = r.data + 2;
  uint16_t mtu = ble_att_mtu(connHandle);
  if (mtu < MIN_MTU) return respond(op, id, ERR_MTU);
  static const uint8_t PAYLOAD_LEN[] = {0, 0, 8, 1, 12, 8, 1, 1};
  if (op < OP_GET_STATUS || op > OP_SET_WAKE_WORD) return respond(op, id, ERR_UNKNOWN_OP);
  if (r.len - 2 != PAYLOAD_LEN[op]) return respond(op, id, ERR_BAD_LENGTH);
  switch (op) {
    case OP_GET_STATUS: return handleStatus(id);
    case OP_SET_TIME:
    {
      uint64_t epochMs = get64(p);
      if (epochMs < MIN_EPOCH_MS) return respond(op, id, ERR_BAD_VALUE);
      epochOffsetMs = (int64_t)epochMs - (int64_t)uptimeMs();
      timeSynced = true;
      headerDirty = true;
      return respond(op, id, ST_OK);
    }
    case OP_LIST:  return handleList(id, p[0], mtu);
    case OP_PULL:  return handlePull(id, p, mtu);
    case OP_ACK:   return handleAck(id, p);
    case OP_SET_MUTE:
      if (p[0] > 1) return respond(op, id, ERR_BAD_VALUE);
      muted = p[0];
      prefs.putBool("muted", p[0]);
      Serial.println(p[0] ? "muted" : "unmuted");
      return respond(op, id, ST_OK);
    case OP_SET_WAKE_WORD: return respond(op, id, ERR_UNSUPPORTED);  // no detector yet
  }
}

void bleTask(void *) {
  Request r;
  for (;;) {
    sendEvents();
    if (xQueueReceive(requests, &r, pdMS_TO_TICKS(50)) == pdTRUE) handleRequest(r);
  }
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // off
  prefs.begin("tk");
  bootId = prefs.getUInt("boot_id", 0) + 1;
  prefs.putUInt("boot_id", bootId);
  muted = prefs.getBool("muted", false);
  if (!prefs.getBool("paired", false)) openPairingWindow();  // out of the box: no button press needed
  pinMode(BOOT_PIN, INPUT_PULLUP);
  requests = xQueueCreate(2, sizeof(Request));
  events = xQueueCreate(8, sizeof(Event));
  bookmarks = xQueueCreate(4, sizeof(uint64_t));

  startBle();
  if (!prefs.isKey("store_id")) prefs.putUInt("store_id", esp_random());  // after BLE init: the radio seeds the RNG
  storeId = prefs.getUInt("store_id");
  startStorage();
  Serial.printf("pendant up: store %08lX boot %lu%s\n", (unsigned long)storeId, (unsigned long)bootId, muted ? " (muted)" : "");

  xTaskCreate(micTask, "mic", 4096, nullptr, configMAX_PRIORITIES - 3, nullptr);
  xTaskCreate(writerTask, "writer", 6144, nullptr, 4, nullptr);
  xTaskCreate(bleTask, "ble", 6144, nullptr, 3, &bleTaskHandle);
}

void loop() {
  static uint32_t lastStatusMs = 0;
  while (Serial.available()) {
    if (Serial.read() == 'b') bookmark();
  }
  // Only a deliberate press on the pendant lets a new phone bond, and it forgets the old one: one bond at a time.
  static uint32_t heldSinceMs = 0;
  if (digitalRead(BOOT_PIN) == HIGH) heldSinceMs = 0;
  else if (!heldSinceMs) heldSinceMs = millis() | 1;
  else if (heldSinceMs != UINT32_MAX && millis() - heldSinceMs >= PAIRING_HOLD_MS) {
    heldSinceMs = UINT32_MAX;  // once per press
    if (bleConnected) ble_gap_terminate(connHandle, BLE_ERR_REM_USER_CONN_TERM);
    ble_store_clear();
    openPairingWindow();
    Serial.println("BLE: bonds cleared, pairing window open");
  }
  if (ledOffAtMs && (int32_t)(millis() - ledOffAtMs) >= 0) ledOffAtMs = 0;
  digitalWrite(LED_BUILTIN, pairingWindowOpen() || ledOffAtMs ? LOW : HIGH);  // active LOW: on while pairing
  if (bleConnected && !linkTrusted && millis() - connectedAtMs > (pairingWindowOpen() ? PAIRING_LINK_MS : UNTRUSTED_LINK_MS)) {
    Serial.println("BLE: link never bonded, disconnecting");
    ble_gap_terminate(connHandle, BLE_ERR_REM_USER_CONN_TERM);
    connectedAtMs = millis();  // terminate once, not every loop
  }
  if (millis() - lastStatusMs >= 10000) {
    lastStatusMs = millis();
    Serial.printf("[status] rec=%d chunks=%lu pending=%lu dropped=%lu worst_sd_us=%lu ble=%d%s%s\n", isRecording(),
                  (unsigned long)chunksWritten, (unsigned long)pendingChunks, (unsigned long)droppedSamples,
                  (unsigned long)worstWriteUs.exchange(0), (int)bleConnected, storageFull ? " FULL" : "", sdError ? " SD_ERROR" : "");
  }
  delay(10);
}
