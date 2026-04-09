#include <Arduino.h>

// ─────────────────────────────────────────────
//  Pin & PWM config — same values as your
//  working esc_test.txt, do not change these
// ─────────────────────────────────────────────
const int NUM_MOTORS = 4;
const int escPins[NUM_MOTORS]     = {16, 17, 18, 15};
const int escChannels[NUM_MOTORS] = { 0,  1,  2,  3};

const int PWM_FREQ = 50;
const int PWM_RES  = 12;
const int PWM_MAX  = (1 << PWM_RES) - 1;   // 4095

const int PULSE_MIN = 1000;   // µs — 0 % throttle / arming
const int PULSE_MAX = 2000;   // µs — 100 % throttle
// ─────────────────────────────────────────────

// Exactly the same conversion as your working code
int pulseToDuty(int pulse_us) {
    return map(pulse_us, 0, 20000, 0, PWM_MAX);
}

void setAllMotors(int pulse_us) {
    pulse_us = constrain(pulse_us, PULSE_MIN, PULSE_MAX);
    int duty = pulseToDuty(pulse_us);
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledcWrite(escChannels[i], duty);
    }
}

// Convert 0–100 % to a pulse width and apply it
void setThrottlePct(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    int pulse_us = PULSE_MIN + (int)(pct / 100.0f * (PULSE_MAX - PULSE_MIN));
    setAllMotors(pulse_us);
    Serial.printf("[SET]  %.1f %%  →  %d µs\n", pct, pulse_us);
}

void printHelp() {
    Serial.println("─────────────────────────────────────────");
    Serial.println("  Commands:");
    Serial.println("   0–100     set throttle % (all motors)");
    Serial.println("   s / stop  cut throttle to 0 %");
    Serial.println("   h / help  show this message");
    Serial.println("─────────────────────────────────────────");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== CanSat Motor Tester ===");

    // Identical init to your working code
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledcSetup(escChannels[i], PWM_FREQ, PWM_RES);
        ledcAttachPin(escPins[i], escChannels[i]);
    }

    // Arming: hold min throttle for 5 s (same as your working code)
    Serial.println("Arming — holding min throttle for 5 s...");
    setAllMotors(PULSE_MIN);
    delay(5000);

    Serial.println("Armed. Type a throttle % (0–100) and press Enter.");
    printHelp();
}

void loop() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    if (input.equalsIgnoreCase("s") || input.equalsIgnoreCase("stop")) {
        setAllMotors(PULSE_MIN);
        Serial.println("[STOP] Throttle cut to 0 %");
        return;
    }
    if (input.equalsIgnoreCase("h") || input.equalsIgnoreCase("help")) {
        printHelp();
        return;
    }

    if (!isDigit(input[0]) && input[0] != '.') {
        Serial.printf("[ERR]  Unknown command: \"%s\"\n", input.c_str());
        return;
    }

    float pct = input.toFloat();
    if (pct < 0.0f || pct > 100.0f) {
        Serial.printf("[ERR]  %.1f %% is out of range, enter 0–100\n", pct);
        return;
    }

    setThrottlePct(pct);
}