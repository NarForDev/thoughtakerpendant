// Host test for chunk.h: ADPCM round trip through an independent reference decoder, plus the header layout.
// Build and run (MSVC):  cl /nologo /EHsc /W4 chunk_test.cpp && chunk_test.exe out.wav out_ref.pcm
// The optional files let ffmpeg cross-check the decoder:  ffmpeg -i out.wav -f s16le ffmpeg.pcm, then compare.
#define _CRT_SECURE_NO_WARNINGS  // MSVC: plain fopen is fine here
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "../chunk.h"

static int failures = 0;
#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
      failures++;                                                  \
    }                                                              \
  } while (0)

// Decoder written from the Microsoft IMA ADPCM description, independent of the encoder's code.
static void referenceDecodeBlock(const uint8_t *in, int16_t *out) {
  static const int indexTable[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};
  int pred = (int16_t)(in[0] | in[1] << 8);
  int index = in[2];
  out[0] = (int16_t)pred;
  for (uint32_t i = 1; i < BLOCK_SAMPLES; i++) {
    uint8_t byte = in[4 + (i - 1) / 2];
    int n = ((i - 1) & 1) ? byte >> 4 : byte & 15;
    int step = ADPCM_STEPS[index];
    int diff = step >> 3;
    if (n & 4) diff += step;
    if (n & 2) diff += step >> 1;
    if (n & 1) diff += step >> 2;
    pred = (n & 8) ? pred - diff : pred + diff;
    if (pred > 32767) pred = 32767;
    if (pred < -32768) pred = -32768;
    index += indexTable[n];
    if (index < 0) index = 0;
    if (index > 88) index = 88;
    out[i] = (int16_t)pred;
  }
}

static double snrDb(const int16_t *ref, const int16_t *got, size_t n) {
  double sig = 0, err = 0;
  for (size_t i = 0; i < n; i++) {
    sig += (double)ref[i] * ref[i];
    err += ((double)ref[i] - got[i]) * ((double)ref[i] - got[i]);
  }
  return err == 0 ? 999 : 10 * log10(sig / err);
}

int main(int argc, char **argv) {
  const double PI = 3.14159265358979;
  // 1 s each: 440 Hz sine at -6 dBFS, speech-band chirp, silence, full-scale square (clipping stress), noise.
  const uint32_t segment = SAMPLE_RATE, blocks = 5 * segment / BLOCK_SAMPLES + 1;
  std::vector<int16_t> pcm(blocks * BLOCK_SAMPLES, 0), decoded(pcm.size());
  srand(1);
  for (uint32_t i = 0; i < pcm.size(); i++) {
    double t = (double)i / SAMPLE_RATE;
    switch (i / segment) {
      case 0: pcm[i] = (int16_t)(16384 * sin(2 * PI * 440 * t)); break;
      case 1: pcm[i] = (int16_t)(12000 * sin(2 * PI * (200 + 1800 * (t - 1)) * (t - 1))); break;
      case 2: pcm[i] = 0; break;
      case 3: pcm[i] = (i / 20) % 2 ? 32767 : -32768; break;
      default: pcm[i] = (int16_t)(rand() % 8001 - 4000); break;
    }
  }

  std::vector<uint8_t> audio(blocks * BLOCK_BYTES);
  uint8_t stepIndex = 0;
  for (uint32_t b = 0; b < blocks; b++) adpcmEncodeBlock(&pcm[b * BLOCK_SAMPLES], &audio[b * BLOCK_BYTES], stepIndex);
  // Decode in reverse order: every block must stand alone, which crash recovery relies on.
  for (uint32_t b = blocks; b-- > 0;) referenceDecodeBlock(&audio[b * BLOCK_BYTES], &decoded[b * BLOCK_SAMPLES]);

  for (uint32_t b = 0; b < blocks; b++) {
    CHECK(decoded[b * BLOCK_SAMPLES] == pcm[b * BLOCK_SAMPLES]);  // header sample is exact
    CHECK(audio[b * BLOCK_BYTES + 2] <= 88 && audio[b * BLOCK_BYTES + 3] == 0);
  }
  double sine = snrDb(&pcm[0], &decoded[0], segment), chirp = snrDb(&pcm[segment], &decoded[segment], segment);
  double noise = snrDb(&pcm[4 * segment], &decoded[4 * segment], segment);
  int silenceMax = 0;
  for (uint32_t i = 2 * segment + BLOCK_SAMPLES; i < 3 * segment; i++) silenceMax = abs(decoded[i]) > silenceMax ? abs(decoded[i]) : silenceMax;
  printf("SNR sine %.1f dB, chirp %.1f dB, noise %.1f dB, silence peak %d\n", sine, chirp, noise, silenceMax);
  CHECK(sine > 25);
  CHECK(chirp > 20);
  CHECK(noise > 12);  // white noise is the worst case for 4-bit ADPCM
  CHECK(silenceMax <= 8);

  ChunkHeader c = {};
  c.blocks = blocks;
  c.flags = FLAG_SPEECH | FLAG_BOOKMARKED | FLAG_TIME_SYNCED;
  c.storeId = 0xA1B2C3D4;
  c.seq = 0x1A;
  c.bootId = 7;
  c.startUptimeMs = 123456789012ull;
  c.epochOffsetMs = -5;
  c.crc = 0xDEADBEEF;
  c.bookmarkCount = 2;
  c.bookmarkMs[0] = 1000;
  c.bookmarkMs[1] = 59000;
  uint8_t h[HEADER_BYTES];
  buildHeader(h, c);
  CHECK(get32(h + 4) == 504 + blocks * BLOCK_BYTES);
  CHECK(get32(h + 48) == blocks * BLOCK_SAMPLES);
  CHECK(get32(h + 508) == blocks * BLOCK_BYTES);
  CHECK(memcmp(h + 504, "data", 4) == 0 && get32(h + 56) == 444 && get16(h + 38) == 505);
  CHECK(get32(h + 92) == 0xDEADBEEF && get32(h + 104) == 59000 && h[96] == 2);
  ChunkHeader back;
  CHECK(parseHeader(h, back));
  CHECK(back.blocks == c.blocks && back.flags == c.flags && back.storeId == c.storeId && back.seq == c.seq);
  CHECK(back.bootId == c.bootId && back.startUptimeMs == c.startUptimeMs && back.epochOffsetMs == c.epochOffsetMs);
  CHECK(back.crc == c.crc && back.bookmarkCount == 2 && back.bookmarkMs[1] == 59000);
  h[52] = 'x';
  CHECK(!parseHeader(h, back));
  c.blocks = 0;  // in-progress header: sizes are 0
  buildHeader(h, c);
  CHECK(get32(h + 4) == 0 && get32(h + 48) == 0 && get32(h + 508) == 0);
  CHECK(CHUNK_BLOCKS * BLOCK_SAMPLES == 960005 && HEADER_BYTES + CHUNK_BLOCKS * BLOCK_BYTES == 487168);

  if (argc > 2) {
    c.blocks = blocks;
    buildHeader(h, c);
    FILE *f = fopen(argv[1], "wb");
    fwrite(h, 1, HEADER_BYTES, f);
    fwrite(audio.data(), 1, audio.size(), f);
    fclose(f);
    f = fopen(argv[2], "wb");
    fwrite(decoded.data(), sizeof(int16_t), decoded.size(), f);
    fclose(f);
  }
  printf(failures ? "%d FAILED\n" : "PASS\n", failures);
  return failures != 0;
}
