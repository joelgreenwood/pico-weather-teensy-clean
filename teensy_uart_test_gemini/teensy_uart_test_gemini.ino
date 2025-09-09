// A minimal sketch to test UART communication from the Pico.
// It listens on Serial5 for messages in the format "T=###"
// and prints the result to the USB Serial Monitor.

void setup() {
  // Start the main USB serial port for viewing output
  Serial.begin(115200);
  while (!Serial && millis() < 4000); // Wait for monitor to connect

  // Start the hardware serial port connected to the Pico
  // On Teensy 4.0, Serial5 is on pins 20 (RX) and 21 (TX)
  Serial5.begin(115200);

  Serial.println("--- Teensy UART Test Sketch ---");
  Serial.println("Listening for messages from Pico on Serial5...");
}

void loop() {
  // Continuously check for and process incoming data from the Pico
  pollPicoUart();
}

void pollPicoUart() {
    static char buf[32];
    static uint8_t idx = 0;

    if (Serial5.available()) {
        char c = Serial5.read();

        if (c == '\r') {
            // Ignore carriage return characters
            return;
        }

        if (c == '\n' || idx >= sizeof(buf) - 1) {
            // End of line or buffer full, process the message
            buf[idx] = 0; // Null-terminate the string

            if (idx > 0) { // Make sure we have some data
                Serial.print("Received line from Pico: [");
                Serial.print(buf);
                Serial.println("]");

                int tenths;
                // Try to parse the message
                if (sscanf(buf, "T=%d", &tenths) == 1) {
                    float receivedTemp = tenths / 10.0f;
                    Serial.print("  -> Parsed successfully! Temperature: ");
                    Serial.println(receivedTemp, 1);
                } else {
                    Serial.println("  -> Parsing FAILED. Message not in 'T=###' format.");
                }
            }
            idx = 0; // Reset for the next message
        } else {
            // Add character to the buffer
            buf[idx++] = c;
        }
    }
}