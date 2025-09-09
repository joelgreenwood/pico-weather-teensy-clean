#include <Arduino.h>
#include <PID_v1.h>
#include <FastTouch.h>
#include <Wire.h>

// ---------------- Pins & PWM ----------------
const int ENABLE_PIN = 1;
const int DIR_PIN    = 2;
const int PWM_PIN    = 3;
const int THERM_PIN  = A0;
const int TOUCH_PIN  = 12;
const int YELLOW_LED_PIN = 6;
const int GREEN_LED_PIN  = 7;
const int HEARTBEAT_LED  = LED_BUILTIN;

// ---------------- Constants ----------------
const int TEENSY_I2C_ADDRESS = 0x08;
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

// ---------------- State Variables ----------------
enum ControlMode { MODE_TOUCH, MODE_SERIAL };
ControlMode currentMode = MODE_TOUCH;

float picoTargetTempF = 55.0f;
uint32_t lastUpdate = 0;
bool systemInFaultState = false;
double pidSetpointF, pidInputF, pidOutput;
int outAbsMax = PWM_MAX;
float filtTempF = NAN;
bool currentlyHeating = false;
bool touchIsActive = false;

// ---------------- PID ----------------
double Kp_heat = 600.0, Ki_heat = 300.0, Kd_heat = 150.0;
double Kp_cool = 1200.0, Ki_cool = 800.0, Kd_cool = 300.0;
PID myPID(&pidInputF, &pidOutput, &pidSetpointF, Kp_heat, Ki_heat, Kd_heat, P_ON_M, DIRECT);

const float EMA_ALPHA = 0.8f;

// ---------------- Helper Functions ----------------
inline float cToF(float c) { return c * 9.0f/5.0f + 32.0f; }

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

// ---------------- I2C ----------------
void receiveEvent(int numBytes) {
    if (numBytes == 2) {
        int receivedTempInt = (Wire.read() << 8) | Wire.read();
        picoTargetTempF = receivedTempInt / 10.0f;
        if (Serial) {
            Serial.print("I2C: Target updated to ");
            Serial.print(picoTargetTempF, 1);
            Serial.println(" F");
        }
    } else {
        while (Wire.available()) Wire.read();
    }
}

// ---------------- Serial Command Handler ----------------
void handleSerialCommand(String line) {
    line.trim();
    if (line.length() == 0) return;

    // Switch to touch mode
    if (line.equalsIgnoreCase("touch")) {
        currentMode = MODE_TOUCH;
        driveSignedPwm(0);
        digitalWrite(YELLOW_LED_PIN, LOW);
        digitalWrite(GREEN_LED_PIN, LOW);
        if (Serial) Serial.println("--> Switched to TOUCH mode.");
        return;
    }

    // Test commands like test=20,60,80
    int eq = line.indexOf('=');
    if (eq > 0) {
        String key = line.substring(0, eq); key.trim();
        String val = line.substring(eq+1); val.trim();
        if (key.equalsIgnoreCase("test")) {
            int current_pos = 0;
            int step = 0;
            while (step < 10) {
                int next_comma = val.indexOf(',', current_pos);
                String temp_str = (next_comma == -1) ? val.substring(current_pos) : val.substring(current_pos, next_comma);
                if (temp_str.length() > 0) {
                    float t = temp_str.toFloat();
                    // For testing purposes, just update the setpoint immediately
                    pidSetpointF = t;
                    if (Serial) Serial.print("Test setpoint: "); Serial.println(pidSetpointF);
                }
                if (next_comma == -1) break;
                current_pos = next_comma + 1;
                step++;
            }
        }
        return;
    }

    // Otherwise interpret as single float setpoint
    pidSetpointF = line.toFloat();
    if (Serial) {
        Serial.print("Setpoint set to ");
        Serial.println(pidSetpointF);
    }
}

// ---------------- Setup ----------------
void setup() {
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(HEARTBEAT_LED, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(THERM_PIN, INPUT);

    analogReadResolution(ADC_BITS);
    analogReadAveraging(1);
    analogWriteResolution(PWM_BITS);
    analogWriteFrequency(PWM_PIN, PWM_FREQ_HZ);
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    analogWrite(PWM_PIN, 0);

    pidSetpointF = 72.0f;
    myPID.SetOutputLimits(-PWM_MAX, PWM_MAX);
    myPID.SetSampleTime(SAMPLE_MS);
    myPID.SetMode(AUTOMATIC);

    // I2C
    Wire.begin(TEENSY_I2C_ADDRESS);
    Wire.onReceive(receiveEvent);

    lastUpdate = millis();

    if (Serial) {
        Serial.println("--- Peltier PID Controller (Optional Serial) ---");
    }
}

// ---------------- Loop ----------------
void loop() {
    // Serial commands
    if (Serial && Serial.available()) {
        String line = Serial.readStringUntil('\n');
        handleSerialCommand(line);
    }

    // PID / Touch logic
    uint32_t now = millis();
    if (now - lastUpdate >= SAMPLE_MS) {
        lastUpdate = now;
        bool shouldRunPID = false;

        if (currentMode == MODE_TOUCH) {
            bool prevTouchState = touchIsActive;
            touchIsActive = fastTouchRead(TOUCH_PIN) > TOUCH_THRESHOLD;
            if (touchIsActive) {
                if (!prevTouchState) {
                    myPID.SetMode(MANUAL);
                    myPID.SetMode(AUTOMATIC);
                    filtTempF = NAN;
                }
                shouldRunPID = true;
                pidSetpointF = picoTargetTempF;
            } else {
                driveSignedPwm(0);
                digitalWrite(YELLOW_LED_PIN, LOW);
                digitalWrite(GREEN_LED_PIN, LOW);
            }
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
                driveSignedPwm(0);
            }

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
        }

        digitalWrite(HEARTBEAT_LED, (millis() % 1000) < 50 ? HIGH : LOW);
    }
}
