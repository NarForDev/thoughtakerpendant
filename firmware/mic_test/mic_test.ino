// Bring-up step 3: INMP441 mic over I2S, plus a 5 s WAV on the SD card to listen to.
// L/R is tied to GND, so the mic sends on the left slot.
#include <ESP_I2S.h>
#include <SD.h>

const uint32_t SAMPLE_RATE = 16000;
const size_t WINDOW_SAMPLES = SAMPLE_RATE / 2;  // one meter line per 0.5 s
const char *WAV_PATH = "/mic_test.wav";

I2SClass i2s;
int16_t window[WINDOW_SAMPLES];

void saveTestRecording() {
  if (!SD.begin(D2)) { Serial.println("FAIL: no SD card, skipping WAV"); return; }
  Serial.println("recording 5 s - talk now");
  size_t wavBytes;
  uint8_t *wav = i2s.recordWAV(5, &wavBytes);
  if (!wav) { Serial.println("FAIL: recordWAV (out of RAM?)"); return; }
  File f = SD.open(WAV_PATH, FILE_WRITE);
  bool ok = f && f.write(wav, wavBytes) == wavBytes;
  f.close();
  free(wav);
  Serial.println(ok ? "saved /mic_test.wav" : "FAIL: WAV write");
}

void setup() {
  Serial.begin(115200);
  i2s.setPins(D3, D4, -1, D5);  // SCK, WS, no output, mic data
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)
      || !i2s.configureRX(SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_RX_TRANSFORM_32_TO_16)) {
    Serial.println("FAIL: I2S init");
    while (true) delay(1000);
  }
  saveTestRecording();
}

void loop() {
  size_t got = i2s.readBytes((char *)window, sizeof(window)) / sizeof(int16_t);
  int16_t lo = INT16_MAX, hi = INT16_MIN;
  int64_t sum = 0;
  double sumSq = 0;
  for (size_t i = 0; i < got; i++) {
    lo = min(lo, window[i]);
    hi = max(hi, window[i]);
    sum += window[i];
    sumSq += (double)window[i] * window[i];
  }
  if (got == 0 || lo == hi) {
    Serial.printf("FAIL: no signal (%u samples, stuck at %d) - check SCK/WS/SD/L-R wiring\n", got, lo);
    delay(500);
    return;
  }
  Serial.printf("rms %.1f dBFS, peak %d, dc %lld\n", 20 * log10(sqrt(sumSq / got) / 32768), max(-lo, (int)hi), sum / got);
}
