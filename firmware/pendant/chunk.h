// Chunk file format (docs/protocol.md section 1): IMA ADPCM encoder and the 512-byte header.
// Plain C++ with no Arduino dependencies so test/chunk_test.cpp can check it on the host.
#pragma once
#include <stdint.h>
#include <string.h>

const uint32_t SAMPLE_RATE = 16000;
const uint32_t BLOCK_BYTES = 256;
const uint32_t BLOCK_SAMPLES = 505;
const uint32_t CHUNK_BLOCKS = 1901;
const uint32_t HEADER_BYTES = 512;
const uint32_t MAX_BOOKMARKS = 16;

const uint16_t FLAG_SPEECH = 1, FLAG_BOOKMARKED = 2, FLAG_TIME_SYNCED = 4, FLAG_RECOVERED = 8;

struct ChunkHeader {
  uint32_t blocks;  // 0 while recording: riff size, fact and data size stay 0 until finalize
  uint16_t flags;
  uint32_t storeId, seq, bootId;
  uint64_t startUptimeMs;
  int64_t epochOffsetMs;
  uint32_t crc;
  uint8_t bookmarkCount;
  uint32_t bookmarkMs[MAX_BOOKMARKS];
};

inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }
inline void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)v); put32(p + 4, (uint32_t)(v >> 32)); }
inline uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
inline uint32_t get32(const uint8_t *p) { return get16(p) | (uint32_t)get16(p + 2) << 16; }
inline uint64_t get64(const uint8_t *p) { return get32(p) | (uint64_t)get32(p + 4) << 32; }

inline void buildHeader(uint8_t *h, const ChunkHeader &c) {
  uint32_t dataSize = c.blocks * BLOCK_BYTES;
  memset(h, 0, HEADER_BYTES);
  memcpy(h, "RIFF", 4);
  put32(h + 4, c.blocks ? 504 + dataSize : 0);
  memcpy(h + 8, "WAVEfmt ", 8);
  put32(h + 16, 20);
  put16(h + 20, 0x0011);
  put16(h + 22, 1);
  put32(h + 24, SAMPLE_RATE);
  put32(h + 28, 8110);
  put16(h + 32, BLOCK_BYTES);
  put16(h + 34, 4);
  put16(h + 36, 2);
  put16(h + 38, BLOCK_SAMPLES);
  memcpy(h + 40, "fact", 4);
  put32(h + 44, 4);
  put32(h + 48, c.blocks * BLOCK_SAMPLES);
  memcpy(h + 52, "tkmd", 4);
  put32(h + 56, 444);
  put16(h + 60, 1);
  put16(h + 62, c.flags);
  put32(h + 64, c.storeId);
  put32(h + 68, c.seq);
  put32(h + 72, c.bootId);
  put64(h + 76, c.startUptimeMs);
  put64(h + 84, (uint64_t)c.epochOffsetMs);
  put32(h + 92, c.crc);
  h[96] = c.bookmarkCount;
  for (uint32_t i = 0; i < c.bookmarkCount && i < MAX_BOOKMARKS; i++) put32(h + 100 + 4 * i, c.bookmarkMs[i]);
  memcpy(h + 504, "data", 4);
  put32(h + 508, dataSize);
}

// Returns false unless the header has the RIFF and tkmd markers recovery requires.
inline bool parseHeader(const uint8_t *h, ChunkHeader &c) {
  if (memcmp(h, "RIFF", 4) || memcmp(h + 52, "tkmd", 4)) return false;
  memset(&c, 0, sizeof(c));
  c.blocks = get32(h + 508) / BLOCK_BYTES;
  c.flags = get16(h + 62);
  c.storeId = get32(h + 64);
  c.seq = get32(h + 68);
  c.bootId = get32(h + 72);
  c.startUptimeMs = get64(h + 76);
  c.epochOffsetMs = (int64_t)get64(h + 84);
  c.crc = get32(h + 92);
  c.bookmarkCount = h[96] < MAX_BOOKMARKS ? h[96] : MAX_BOOKMARKS;
  for (uint32_t i = 0; i < c.bookmarkCount; i++) c.bookmarkMs[i] = get32(h + 100 + 4 * i);
  return true;
}

const int16_t ADPCM_STEPS[89] = {
  7,     8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,    28,    31,    34,    37,
  41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,   118,   130,   143,   157,   173,   190,   209,
  230,   253,   279,   307,   337,   371,   408,   449,   494,   544,   598,   658,   724,   796,   876,   963,   1060,  1166,
  1282,  1411,  1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
  7132,  7845,  8630,  9493,  10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};
const int8_t ADPCM_INDEX_SHIFT[8] = {-1, -1, -1, -1, 2, 4, 6, 8};

// Encodes 505 samples into one 256-byte Microsoft IMA ADPCM block. stepIndex carries over between
// blocks so the quantizer stays adapted; the decoder never needs it because each header restates it.
inline void adpcmEncodeBlock(const int16_t *pcm, uint8_t *out, uint8_t &stepIndex) {
  int pred = pcm[0];
  int index = stepIndex;
  put16(out, (uint16_t)pcm[0]);
  out[2] = (uint8_t)index;
  out[3] = 0;
  for (uint32_t i = 1; i < BLOCK_SAMPLES; i++) {
    int step = ADPCM_STEPS[index];
    int diff = pcm[i] - pred;
    int nibble = 0;
    if (diff < 0) {
      nibble = 8;
      diff = -diff;
    }
    // Same shift-and-add the reference decoder uses, so pred tracks the decoder exactly.
    int delta = step >> 3;
    if (diff >= step) { nibble |= 4; diff -= step; delta += step; }
    if (diff >= step >> 1) { nibble |= 2; diff -= step >> 1; delta += step >> 1; }
    if (diff >= step >> 2) { nibble |= 1; delta += step >> 2; }
    pred += (nibble & 8) ? -delta : delta;
    pred = pred < -32768 ? -32768 : pred > 32767 ? 32767 : pred;
    index += ADPCM_INDEX_SHIFT[nibble & 7];
    index = index < 0 ? 0 : index > 88 ? 88 : index;
    uint8_t &b = out[4 + (i - 1) / 2];
    b = ((i - 1) & 1) ? (uint8_t)(b | nibble << 4) : (uint8_t)nibble;  // low nibble = earlier sample
  }
  stepIndex = (uint8_t)index;
}
