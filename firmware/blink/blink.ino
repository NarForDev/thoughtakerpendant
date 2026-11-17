// Bring-up step 1: toolchain, upload, USB serial, and the status LED.
// XIAO ESP32-C6 user LED (GPIO15) is active LOW.

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);  // on
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH); // off
  delay(500);
  Serial.printf("alive %lu ms\n", millis()); // count dropping back to near zero = board rebooted
}
