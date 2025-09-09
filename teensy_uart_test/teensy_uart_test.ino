// Teensy side
float picoTargetTempF = 55.0f;   // gets updated by Pico

void setup() {
  Serial.begin(115200);  // USB serial for debugging
  Serial5.begin(115200); // UART link to Pico on pins 20/21
}

void pollPicoUart() {
  static char buf[16];
  static uint8_t idx = 0;

  while (Serial5.available()) {
    char c = Serial5.read();
    if (c == '\n' || idx >= sizeof(buf)-1) {
      buf[idx] = 0;
      int tenths;
      if (sscanf(buf, "T=%d", &tenths) == 1) {
        picoTargetTempF = tenths / 10.0f;
        Serial.print("Got temp from Pico: ");
        Serial.println(picoTargetTempF);
      }
      idx = 0;
    } else {
      buf[idx++] = c;
    }
  }
}

void loop() {
  pollPicoUart();
  // ... rest of your control loop ...
}
