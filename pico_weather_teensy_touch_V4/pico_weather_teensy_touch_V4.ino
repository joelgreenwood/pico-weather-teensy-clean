#include <Arduino.h>
#include <PID_v1.h>
#include <FastTouch.h>

// ---------------- Pins & PWM ----------------
const int ENABLE_PIN = 1;
const int DIR_PIN    = 2;
const int PWM_PIN    = 3;
const int THERM_PIN  = A0;
const int TOUCH_PIN  = 12;
const int YELLOW_LED_PIN = 6; // "Working" indicator
const int GREEN_LED_PIN  = 7; // "On Target" indicator
const int HEARTBEAT_LED  = LED_BUILTIN;

// ---------------- Constants ----------------
const int ADC_BITS = 12;
const int ADC_MAX  = (1 << ADC_BITS) - 1;
const int PWM_BITS = 12;
const int PWM_MAX  = (1 << ADC_BITS) - 1;
int PWM_FREQ_HZ = 50;
const float VREF    = 3.3f;
float R_FIXED       = 10000.0f;
const float R0      = 10000.0f;
const float T0_K    = 298.15f;
const float BETA    = 3950.0f;
const uint32_t SAMPLE_MS = 100;
const int ADC_SAMPLES = 16;
const float SAFETY_MAX_TEMP_F = 130.0f;
const int TOUCH_THRESHOLD = 30;
const float TEMP_OK_BAND_F = 1.0f;
const float EMA_ALPHA = 0.8f;

// ---------------- State Variables ----------------
enum ControlMode { MODE_TOUCH, MODE_SERIAL };
ControlMode currentMode = MODE_TOUCH; // Default to touch mode on boot

float picoTargetTempF = 55.0f; // Stores temp from Pico, with a safe default

uint32_t lastUpdate = 0;
bool systemInFaultState = false;
double pidSetpointF, pidInputF, pidOutput;
int outAbsMax = PWM_MAX;
float filtTempF = NAN;
bool currentlyHeating = false;
bool touchIsActive = false;

// --------- Logging / test harness ----------
bool loggingEnabled = true; // ON by default
bool uartRxLoggingEnabled = true; // UART RX messages are ON by default
uint32_t LOG_PERIOD_MS = 500; // Log every half-second
uint32_t lastLogAt = 0;
enum TestState { TEST_IDLE, TEST_RUNNING, TEST_DONE };
TestState testState = TEST_IDLE;
float test_setpoints[10];
int num_test_steps = 0;
int current_test_step = 0;
uint32_t test_step_duration_ms = 30000;
uint32_t test_step_start_ms = 0;
bool picoDataReceivedFlag = false; // Flag for heartbeat LED
uint32_t ledFlashStartMs = 0;      // Timer for heartbeat flash

// ---------------- PID ----------------
double Kp_heat = 600.0, Ki_heat = 300.0, Kd_heat = 150.0;
double Kp_cool = 1200.0, Ki_cool = 800.0, Kd_cool = 300.0;
PID myPID(&pidInputF, &pidOutput, &pidSetpointF, Kp_heat, Ki_heat, Kd_heat, P_ON_M, DIRECT);

// ---------------- Helper Functions ----------------
float readVoltage() {
  uint32_t acc = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) acc += analogRead(THERM_PIN);
  return (acc / float(ADC_SAMPLES) / ADC_MAX) * VREF;
}
float ntcResistanceFromV(float vOut) {
  if (vOut < 0.0002f) vOut = 0.0002f;
  if (vOut > VREF - 0.0002f) vOut = VREF - 0.0002f;
  return R_FIXED * ((VREF / vOut) - 1.0f);
}
float betaCelsiusFromR(float r_ohm) {
  return (1.0f / ((1.0f / T0_K) + (1.0f / BETA) * logf(r_ohm / R0))) - 273.15f;
}
inline float cToF(float c) { return c * 9.0f/5.0f + 32.0f; }
void driveSignedPwm(int signedCmd) {
  int mag = abs(signedCmd);
  if (mag < 20) {
    analogWrite(PWM_PIN, 0);
    digitalWrite(ENABLE_PIN, LOW);
    return;
  }
  if (mag > outAbsMax) mag = outAbsMax;
  bool isHeating = (signedCmd >= 0);
  digitalWrite(DIR_PIN, isHeating ? HIGH : LOW);
  digitalWrite(ENABLE_PIN, HIGH);
  analogWrite(PWM_PIN, mag);
}

// ---------------- UART from Pico (Serial5) ----------------
void pollPicoUart() {
    static char buf[32];
    static uint8_t idx = 0;
    while (Serial5.available()) {
        char c = Serial5.read();
        if (c == '\r') continue; // Ignore carriage return
        if (c == '\n' || idx >= sizeof(buf)-1) {
            buf[idx] = 0; // Null terminate
            int tenths;
            if (sscanf(buf, "T=%d", &tenths) == 1) {
                picoTargetTempF = tenths / 10.0f;
                picoDataReceivedFlag = true; // Set flag for LED heartbeat
                if (uartRxLoggingEnabled) { // Check if UART logging is enabled
                    Serial.print("UART RX: Target updated to ");
                    Serial.print(picoTargetTempF, 1);
                    Serial.println(" F");
                }
            }
            idx = 0;
        } else {
            buf[idx++] = c;
        }
    }
}

// ---------------- USB Serial Command Handler ----------------
void startTest() {
  if (num_test_steps == 0) return;
  current_test_step = 0;
  pidSetpointF = test_setpoints[current_test_step];
  testState = TEST_RUNNING;
  test_step_start_ms = millis();
  loggingEnabled = true;
  Serial.println("# MULTI-STEP TEST START");
  Serial.print("# Each step duration: "); Serial.print(test_step_duration_ms / 1000.0f); Serial.println(" seconds");
  Serial.print("# Step 1: setpoint -> "); Serial.println(pidSetpointF);
}

void handleSerialCommand(String line) {
  line.trim();
  if (!line.length()) return;

  if (line.equalsIgnoreCase("touch")) {
    currentMode = MODE_TOUCH;
    driveSignedPwm(0);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    Serial.println("--> Switched to TOUCH mode.");
    return;
  }
  
  if (line.equalsIgnoreCase("status")) {
    Serial.print("--- STATUS (");
    if(currentMode == MODE_SERIAL) Serial.print("Serial");
    else Serial.print("Touch");
    Serial.println(" Mode) ---");
    Serial.print("Active Setpoint: "); Serial.print(pidSetpointF, 2); Serial.println(" F");
    Serial.print("Pico's Target Temp: "); Serial.print(picoTargetTempF, 2); Serial.println(" F");
    Serial.print("UART RX Logging: "); Serial.println(uartRxLoggingEnabled ? "ON" : "OFF");
    return;
  }

  if (line.startsWith("log=")) {
      loggingEnabled = (line.substring(4).toInt() == 1);
      Serial.print("--> Logging "); Serial.println(loggingEnabled ? "ON" : "OFF");
      return;
  }

  if (line.startsWith("uartlog=")) {
      uartRxLoggingEnabled = (line.substring(8).toInt() == 1);
      Serial.print("--> UART RX Logging "); Serial.println(uartRxLoggingEnabled ? "ON" : "OFF");
      return;
  }

  if (line.startsWith("test=")) {
      currentMode = MODE_SERIAL; // This command takes control
      String params = line.substring(5);
      num_test_steps = 0;
      int current_pos = 0;
      int next_comma = params.indexOf(',');
      if (next_comma == -1) { Serial.println("Invalid format."); return; }
      
      test_step_duration_ms = (uint32_t)(params.substring(0, next_comma).toFloat() * 1000.0f);
      current_pos = next_comma + 1;

      while (num_test_steps < 10 && (unsigned int)current_pos < params.length()) {
          next_comma = params.indexOf(',', current_pos);
          String temp_str = (next_comma == -1) ? params.substring(current_pos) : params.substring(current_pos, next_comma);
          if (temp_str.length() > 0) {
              test_setpoints[num_test_steps++] = temp_str.toFloat();
          }
          if (next_comma == -1) break;
          current_pos = next_comma + 1;
      }
      startTest();
      return;
  }
  
  // If it's a number, it's a command that takes control
  float num = line.toFloat();
  if (num != 0.0f || line.equals("0")) {
      currentMode = MODE_SERIAL;
      pidSetpointF = num;
      Serial.print("--> Serial mode activated. Setpoint: ");
      Serial.println(pidSetpointF, 1);
      return;
  }

  Serial.println("Unknown command.");
}

// ---------------- Setup ----------------------
void setup() {
  pinMode(YELLOW_LED_PIN, OUTPUT); pinMode(GREEN_LED_PIN, OUTPUT); pinMode(HEARTBEAT_LED, OUTPUT);
  Serial.begin(115200);
  Serial5.begin(115200); // For Pico communication
  
  pinMode(ENABLE_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT); pinMode(PWM_PIN, OUTPUT); pinMode(THERM_PIN, INPUT);
  analogReadResolution(ADC_BITS); analogReadAveraging(1);
  analogWriteResolution(PWM_BITS); analogWriteFrequency(PWM_PIN, PWM_FREQ_HZ);
  digitalWrite(ENABLE_PIN, LOW); digitalWrite(DIR_PIN, LOW); analogWrite(PWM_PIN, 0);
  
  pidSetpointF = 72.0f; // Default for serial mode
  myPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
  myPID.SetSampleTime(SAMPLE_MS);
  myPID.SetMode(AUTOMATIC);
  
  Serial.println("\n--- Peltier PID Controller ---");
  Serial.println("Starting in TOUCH mode. Listening for Pico UART updates.");
  lastUpdate = millis();
}

// ---------------- Loop -----------------------
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleSerialCommand(line);
  }
  pollPicoUart();

  if (systemInFaultState) {
    driveSignedPwm(0);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(HEARTBEAT_LED, HIGH);
    return;
  }

  uint32_t now = millis();
  if (now - lastUpdate >= SAMPLE_MS) {
    lastUpdate = now;
    
    bool shouldRunPID = false;
    
    if (currentMode == MODE_TOUCH) {
        bool prevTouchState = touchIsActive;
        touchIsActive = fastTouchRead(TOUCH_PIN) > TOUCH_THRESHOLD;

        if (touchIsActive) {
            if (!prevTouchState) {
                myPID.SetMode(MANUAL); myPID.SetMode(AUTOMATIC);
                filtTempF = NAN;
            }
            shouldRunPID = true;
            pidSetpointF = picoTargetTempF;
        } else {
            driveSignedPwm(0);
            digitalWrite(YELLOW_LED_PIN, LOW);
            digitalWrite(GREEN_LED_PIN, LOW);
        }
    } else { // MODE_SERIAL
        shouldRunPID = true;
    }

    if (shouldRunPID) {
        float v = readVoltage();
        float r = ntcResistanceFromV(v);
        float rawTempF = cToF(betaCelsiusFromR(r));

        if (isnan(filtTempF)) filtTempF = rawTempF;
        else filtTempF = EMA_ALPHA * rawTempF + (1.0f - EMA_ALPHA) * filtTempF;

        bool onTarget = abs(filtTempF - pidSetpointF) < TEMP_OK_BAND_F;
        digitalWrite(YELLOW_LED_PIN, onTarget ? LOW : HIGH);
        digitalWrite(GREEN_LED_PIN, onTarget ? HIGH : LOW);

        if (filtTempF > SAFETY_MAX_TEMP_F) {
          systemInFaultState = true;
          Serial.println("!!! SAFETY SHUTDOWN: Temperature > 130F !!!");
          return;
        }
        if (myPID.GetMode() == MANUAL && !systemInFaultState) myPID.SetMode(AUTOMATIC);
        
        if (filtTempF < pidSetpointF) {
          currentlyHeating = true;
          myPID.SetTunings(Kp_heat, Ki_heat, Kd_heat);
        } else {
          currentlyHeating = false;
          myPID.SetTunings(Kp_cool, Ki_cool, Kd_cool);
        }
        
        pidInputF = filtTempF;
        myPID.Compute();
        driveSignedPwm(pidOutput);

        uint32_t now_ms = millis();
        if (testState == TEST_RUNNING && currentMode == MODE_SERIAL) {
            if (now_ms - test_step_start_ms >= test_step_duration_ms) {
                current_test_step++;
                if (current_test_step < num_test_steps) {
                    pidSetpointF = test_setpoints[current_test_step];
                    test_step_start_ms = now_ms;
                    Serial.print("# Step "); Serial.print(current_test_step + 1);
                    Serial.print(": setpoint -> "); Serial.println(pidSetpointF);
                } else {
                    testState = TEST_IDLE;
                    currentMode = MODE_TOUCH; // Revert to touch mode
                    driveSignedPwm(0); // Turn off Peltier
                    digitalWrite(YELLOW_LED_PIN, LOW);
                    digitalWrite(GREEN_LED_PIN, LOW);
                    Serial.println("# TEST DONE. Reverting to TOUCH mode.");
                }
            }
        }

        if (loggingEnabled && (now_ms - lastLogAt >= LOG_PERIOD_MS)) {
            lastLogAt = now_ms;
            Serial.print("Mode: ");
            Serial.print(currentMode == MODE_TOUCH ? "Touch" : "Serial");
            Serial.print(", Set: "); Serial.print(pidSetpointF, 1);
            Serial.print(", Measured: "); Serial.println(filtTempF, 1);
        }
    }

    // --- New Heartbeat Logic ---
    if (picoDataReceivedFlag) {
        digitalWrite(HEARTBEAT_LED, HIGH);
        ledFlashStartMs = millis();
        picoDataReceivedFlag = false; // Consume the flag
    }
    // Turn off the LED after a short flash
    if (ledFlashStartMs != 0 && millis() - ledFlashStartMs > 50) { // 50ms flash
        digitalWrite(HEARTBEAT_LED, LOW);
        ledFlashStartMs = 0;
    }
  }
}

