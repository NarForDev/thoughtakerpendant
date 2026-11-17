// Bring-up step 2: SD card over SPI. CS is D2, not the core default (D3 is the mic's SCK).
#include <SPI.h>
#include <SD.h>

const char *TEST_PATH = "/sd_test.bin";
const size_t BLOCK_BYTES = 512;
const size_t TEST_BLOCKS = 2048;  // 1 MB

void runTest() {
  if (!SD.begin(D2) || SD.cardType() == CARD_NONE) {  // begin() is a no-op once mounted
    SD.end();
    Serial.println("FAIL: no card - check wiring, 5V, and FAT32 format");
    return;
  }
  Serial.printf("card type %d, size %llu MB\n", SD.cardType(), SD.cardSize() / (1024 * 1024));

  uint8_t block[BLOCK_BYTES], readBack[BLOCK_BYTES];
  for (size_t i = 0; i < BLOCK_BYTES; i++) block[i] = i;

  File f = SD.open(TEST_PATH, FILE_WRITE);
  if (!f) { Serial.println("FAIL: open for write"); return; }
  uint32_t worstUs = 0, startMs = millis();
  for (size_t n = 0; n < TEST_BLOCKS; n++) {
    uint32_t t = micros();
    if (f.write(block, BLOCK_BYTES) != BLOCK_BYTES) { Serial.println("FAIL: short write"); f.close(); return; }
    worstUs = max(worstUs, micros() - t);
  }
  f.close();
  uint32_t elapsedMs = millis() - startMs;
  Serial.printf("write %u KB in %lu ms (%lu KB/s), worst block %lu us\n",
                TEST_BLOCKS / 2, elapsedMs, TEST_BLOCKS * 500 / elapsedMs, worstUs);

  f = SD.open(TEST_PATH);
  if (!f) { Serial.println("FAIL: open for read"); return; }
  for (size_t n = 0; n < TEST_BLOCKS; n++) {
    if (f.read(readBack, BLOCK_BYTES) != BLOCK_BYTES || memcmp(block, readBack, BLOCK_BYTES)) {
      Serial.printf("FAIL: readback mismatch at block %u\n", n);
      f.close();
      return;
    }
  }
  f.close();
  SD.remove(TEST_PATH);
  Serial.println("PASS");
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  runTest();
  delay(5000);
}
