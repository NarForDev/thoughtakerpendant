// Bring-up step 5: can the C6 run streaming wake-word models on live mic audio, and how fast?
// Uses published microWakeWord models (okay_nabu, vad) before we train "Perthro".
#include <ESP_I2S.h>
#include <tensorflow/lite/micro/micro_allocator.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/micro/micro_resource_variable.h>
#include "src/microfrontend/src/frontend.h"
#include "src/microfrontend/src/frontend_util.h"
#include "models.h"

const uint32_t SAMPLE_RATE = 16000;
const size_t STEP_SAMPLES = 160;  // 10 ms feature step, as the models were trained
const int FEATURE_COUNT = 40;
const uint32_t REPORT_MS = 5000;

struct StreamingModel {
  const char *name;
  const unsigned char *flatbuffer;
  size_t arenaBytes;
  uint8_t cutoff;  // probability_cutoff from the model's json, scaled to 0..255
  tflite::MicroInterpreter *interpreter;
  int stride, filled;
  uint8_t recent[5];  // sliding_window_size from the model's json
  uint32_t calls, hits, worstUs, totalUs;
};

StreamingModel models[] = {
  {"okay_nabu", okay_nabu_tflite, 26080 + 4096, (uint8_t)(0.97 * 255)},
  {"vad", vad_tflite, 16772 + 4096, (uint8_t)(0.5 * 255)},
};

I2SClass i2s;
FrontendState frontend;
tflite::MicroMutableOpResolver<20> resolver;

void registerOps() {  // the op set microWakeWord's streaming models use
  resolver.AddCallOnce(); resolver.AddVarHandle(); resolver.AddReadVariable(); resolver.AddAssignVariable();
  resolver.AddReshape(); resolver.AddStridedSlice(); resolver.AddConcatenation(); resolver.AddConv2D();
  resolver.AddDepthwiseConv2D(); resolver.AddMul(); resolver.AddAdd(); resolver.AddMean();
  resolver.AddFullyConnected(); resolver.AddLogistic(); resolver.AddQuantize(); resolver.AddAveragePool2D();
  resolver.AddMaxPool2D(); resolver.AddPad(); resolver.AddPack(); resolver.AddSplitV();
}

bool setupModel(StreamingModel &m) {
  uint8_t *arena = (uint8_t *)malloc(m.arenaBytes);
  if (!arena) return false;
  auto *allocator = tflite::MicroAllocator::Create(arena, m.arenaBytes);
  auto *vars = tflite::MicroResourceVariables::Create(allocator, 20);
  m.interpreter = new tflite::MicroInterpreter(tflite::GetModel(m.flatbuffer), resolver, allocator, vars);
  if (m.interpreter->AllocateTensors() != kTfLiteOk) return false;
  m.stride = m.interpreter->input(0)->dims->data[1];
  Serial.printf("%s: arena %u of %u B, stride %d\n", m.name, m.interpreter->arena_used_bytes(), m.arenaBytes, m.stride);
  return true;
}

void setupFrontend() {  // must match microWakeWord's training features
  FrontendConfig c;
  FrontendFillConfigWithDefaults(&c);
  c.window.size_ms = 30;
  c.window.step_size_ms = 10;
  c.filterbank.num_channels = FEATURE_COUNT;
  c.filterbank.lower_band_limit = 125.0;
  c.filterbank.upper_band_limit = 7500.0;
  c.noise_reduction.smoothing_bits = 10;
  c.noise_reduction.even_smoothing = 0.025;
  c.noise_reduction.odd_smoothing = 0.06;
  c.noise_reduction.min_signal_remaining = 0.05;
  c.pcan_gain_control.enable_pcan = 1;
  c.pcan_gain_control.strength = 0.95;
  c.pcan_gain_control.offset = 80.0;
  c.pcan_gain_control.gain_bits = 21;
  c.log_scale.enable_log = 1;
  c.log_scale.scale_shift = 6;
  FrontendPopulateState(&c, &frontend, SAMPLE_RATE);
}

void feed(StreamingModel &m, const int8_t *features) {
  memcpy(m.interpreter->input(0)->data.int8 + m.filled * FEATURE_COUNT, features, FEATURE_COUNT);
  if (++m.filled < m.stride) return;
  m.filled = 0;
  uint32_t t = micros();
  m.interpreter->Invoke();
  t = micros() - t;
  m.calls++;
  m.totalUs += t;
  m.worstUs = max(m.worstUs, t);
  memmove(m.recent, m.recent + 1, sizeof m.recent - 1);
  m.recent[sizeof m.recent - 1] = m.interpreter->output(0)->data.uint8[0];
  uint32_t sum = 0;
  for (uint8_t p : m.recent) sum += p;
  if (sum / sizeof m.recent > m.cutoff) {
    m.hits++;
    if (m.flatbuffer == okay_nabu_tflite) Serial.println("DETECTED okay_nabu");
    memset(m.recent, 0, sizeof m.recent);
  }
}

void setup() {
  Serial.begin(115200);
  i2s.setPins(D3, D4, -1, D5);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)
      || !i2s.configureRX(SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO, I2S_RX_TRANSFORM_32_TO_16)) {
    Serial.println("FAIL: I2S init");
    while (true) delay(1000);
  }
  registerOps();
  setupFrontend();
  for (auto &m : models) {
    if (!setupModel(m)) {
      Serial.printf("FAIL: %s did not load\n", m.name);
      while (true) delay(1000);
    }
  }
}

void loop() {
  static int16_t samples[STEP_SAMPLES];
  static int8_t features[FEATURE_COUNT];
  static uint32_t lastReportMs, frontendUs;
  i2s.readBytes((char *)samples, sizeof samples);
  uint32_t t = micros();
  size_t used;
  FrontendOutput out = FrontendProcessSamples(&frontend, samples, STEP_SAMPLES, &used);
  frontendUs += micros() - t;
  if (!out.size) return;
  for (int i = 0; i < FEATURE_COUNT; i++) {  // same scaling microWakeWord was trained with: ~0..670 -> int8
    int32_t v = ((int32_t)out.values[i] * 256 + 333) / 666 - 128;
    features[i] = (int8_t)constrain(v, -128, 127);
  }
  for (auto &m : models) feed(m, features);
  if (millis() - lastReportMs < REPORT_MS) return;
  lastReportMs = millis();
  Serial.printf("frontend %.1f%% CPU, free heap %u B\n", frontendUs / (REPORT_MS * 10.0), ESP.getFreeHeap());
  frontendUs = 0;
  for (auto &m : models) {
    Serial.printf("  %s: %.1f%% CPU, avg %lu us, worst %lu us, hits %lu\n", m.name, m.totalUs / (REPORT_MS * 10.0),
                  m.calls ? m.totalUs / m.calls : 0, m.worstUs, m.hits);
    m.calls = m.hits = m.worstUs = m.totalUs = 0;
  }
}
