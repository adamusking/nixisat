/**
 * NixiSat — Semifinal demonstration code
 * ESP32-S3, PlatformIO, Arduino framework
 *
 * lib_deps:
 *   arduino-libraries/Madgwick
 *   hideakitai/MPU9250
 *
 * ─── Modes ───────────────────────────────────────────────────────────────────
 *
 *  After 5 s arming + IMU check, one of two modes runs automatically:
 *
 *  MODE_STABILIZED (IMU detected):
 *    - Ramps to 20% throttle over ~3 s
 *    - Stabilization PID runs continuously
 *    - Tilt the cansat by hand → opposite motors spin up to resist the tilt
 *    - After ~10 s ramps back down to 5% and holds
 *    - Good for in-hand demonstration to judges
 *
 *  MODE_NO_IMU (IMU not detected):
 *    - Simple throttle sweep: 0% → 50% → 0% over 10 s
 *    - No stabilization, all motors identical
 *    - Fallback to still show motors working
 *
 *  To FORCE a mode regardless of IMU detection, change FORCE_MODE below.
 *
 * ─── How to switch modes ─────────────────────────────────────────────────────
 *
 *  Automatic (default, FORCE_MODE = MODE_AUTO):
 *    Power on → 5 s arm → IMU check → mode selected automatically.
 *    This is what you want for the demo.
 *
 *  Force stabilized (for bench testing with IMU, skips auto-detect wait):
 *    Set FORCE_MODE = MODE_STABILIZED
 *
 *  Force no-IMU sweep (for ESC/motor testing without IMU connected):
 *    Set FORCE_MODE = MODE_NO_IMU
 *
 *  The serial monitor always prints which mode is active and why.
 *
 * ─── PID tuning reminder ─────────────────────────────────────────────────────
 *
 *  ALL GAINS ARE ZERO — this is intentional for the in-hand demo.
 *  At zero gains the motors run at BASE throttle regardless of orientation,
 *  which is safe for a hand-held demonstration.
 *
 *  To see stabilization response during the demo, increase ROLL_KP and
 *  PITCH_KP slightly (try 1.0–2.0). The cansat is hand-held so there is
 *  no crash risk at low gains. Do NOT use high gains hand-held — the
 *  corrective thrust will push against your hand unpredictably.
 *
 *  For test-rig tuning, use hover_controller.cpp instead.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>

// ═══════════════════════════════════════════════════════════════════════════
// USER CONFIG
// ═══════════════════════════════════════════════════════════════════════════

// Mode selection — change this to override automatic detection
#define MODE_AUTO           0    // detect IMU, pick mode automatically
#define MODE_STABILIZED     1    // always run stabilization (IMU must be present)
#define MODE_NO_IMU         2    // always run throttle sweep (no IMU needed)

#define FORCE_MODE          MODE_AUTO   // <── change this to force a mode

// I2C
#define I2C_SDA             8
#define I2C_SCL             9
#define MPU_ADDR            0x68

// Sample rate
#define SAMPLE_RATE_HZ      100
#define SAMPLE_INTERVAL_US  (1000000 / SAMPLE_RATE_HZ)
#define DT                  (1.0f / SAMPLE_RATE_HZ)

// Madgwick
#define MADGWICK_BETA       0.1f

// IMU bias — paste calibration values here when available
#define GYRO_BIAS_X         0.0f
#define GYRO_BIAS_Y         0.0f
#define GYRO_BIAS_Z         0.0f
#define ACCEL_BIAS_X        0.0f
#define ACCEL_BIAS_Y        0.0f
#define ACCEL_BIAS_Z        0.0f

// ── ESC / PWM — kept exactly as your propeller_testing.cpp ──────────────────
const int NUM_MOTORS              = 4;
const int escPins[NUM_MOTORS]     = {16, 17, 18, 15};
const int escChannels[NUM_MOTORS] = { 0,  1,  2,  3};

const int PWM_FREQ   = 50;
const int PWM_RES    = 12;
const int PWM_MAX    = (1 << PWM_RES) - 1;   // 4095
const int PULSE_MIN  = 1000;                   // µs — 0% / arming
const int PULSE_MAX  = 2000;                   // µs — 100%

// ── Demo throttle targets ────────────────────────────────────────────────────
// MODE_STABILIZED: ramp to DEMO_HIGH_PCT, hold, then ramp to DEMO_HOLD_PCT
#define DEMO_HIGH_PCT       15.0f   // % — peak throttle during stabilized demo
#define DEMO_HOLD_PCT        5.0f   // % — hold throttle after ramp-down
#define DEMO_RAMP_UP_MS    3000     // ms — time to ramp from 0 to DEMO_HIGH_PCT
#define DEMO_HOLD_HIGH_MS  10000    // ms — how long to hold at DEMO_HIGH_PCT
#define DEMO_RAMP_DOWN_MS  3000     // ms — time to ramp down to DEMO_HOLD_PCT

// MODE_NO_IMU: sweep 0% → SWEEP_PEAK_PCT → 0% over SWEEP_DURATION_MS
#define SWEEP_PEAK_PCT      50.0f
#define SWEEP_DURATION_MS   10000

// ── PID gains ────────────────────────────────────────────────────────────────
// Zero = safe for hand-held demo (motors don't fight your grip).
// Increase ROLL_KP / PITCH_KP to 1.0–2.0 to show visible stabilization response.
#define ROLL_KP             1.5f
#define ROLL_KI             0.0f
#define ROLL_KD             0.0f
#define PITCH_KP            1.5f
#define PITCH_KI            0.0f
#define PITCH_KD            0.0f
#define YAW_KP              0.0f
#define YAW_KI              0.0f
#define YAW_KD              0.0f

#define PID_OUTPUT_LIMIT    200     // µs max correction per axis
#define INTEGRAL_LIMIT      100     // µs anti-windup clamp

// Safety: if tilt exceeds this, zero PID output (not cut motors — hand-held)
#define MAX_SAFE_ANGLE_DEG  180.0f

// ═══════════════════════════════════════════════════════════════════════════
// PID CONTROLLER
// ═══════════════════════════════════════════════════════════════════════════

struct PIDController {
    float kp, ki, kd;
    float integral;
    float outputLimit;
    float integralLimit;

    PIDController(float p, float i, float d, float outLim, float intLim)
        : kp(p), ki(i), kd(d), integral(0.0f),
          outputLimit(outLim), integralLimit(intLim) {}

    float update(float setpoint, float measured, float gyroRate, float dt) {
        float error = setpoint - measured;
        float p     = kp * error;

        integral = constrain(integral + error * dt, -integralLimit, integralLimit);
        float i  = ki * integral;

        // D uses raw gyro rate directly — avoids differentiating noisy error signal
        float d  = -kd * gyroRate;

        return constrain(p + i + d, -outputLimit, outputLimit);
    }

    void reset() { integral = 0.0f; }
};

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════════════

MPU9250  mpu;
Madgwick madgwick;

PIDController pidRoll (ROLL_KP,  ROLL_KI,  ROLL_KD,  PID_OUTPUT_LIMIT, INTEGRAL_LIMIT);
PIDController pidPitch(PITCH_KP, PITCH_KI, PITCH_KD, PID_OUTPUT_LIMIT, INTEGRAL_LIMIT);
PIDController pidYaw  (YAW_KP,   YAW_KI,   YAW_KD,   PID_OUTPUT_LIMIT, INTEGRAL_LIMIT);

volatile bool sampleReady = false;
hw_timer_t*   sampleTimer = nullptr;

int  activeMode     = MODE_AUTO;
bool imuAvailable   = false;

// ═══════════════════════════════════════════════════════════════════════════
// PWM HELPERS — same logic as propeller_testing.cpp
// ═══════════════════════════════════════════════════════════════════════════

int pulseToDuty(int pulse_us) {
    return map(pulse_us, 0, 20000, 0, PWM_MAX);
}

void setMotorPulse(int channel, int pulse_us) {
    pulse_us = constrain(pulse_us, PULSE_MIN, PULSE_MAX);
    ledcWrite(channel, pulseToDuty(pulse_us));
}

void setAllMotors(int pulse_us) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        setMotorPulse(escChannels[i], pulse_us);
    }
}

// Convert 0–100% to pulse and apply to all motors equally
void setAllThrottlePct(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    int pulse_us = PULSE_MIN + (int)(pct / 100.0f * (PULSE_MAX - PULSE_MIN));
    setAllMotors(pulse_us);
}

// Convert % to pulse width in µs — used by mixer
int pctToPulse(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    return PULSE_MIN + (int)(pct / 100.0f * (PULSE_MAX - PULSE_MIN));
}

// Linearly interpolate throttle from startPct to endPct over durationMs.
// Blocking — only used during startup ramp sequences, not in the control loop.
void rampThrottle(float startPct, float endPct, uint32_t durationMs) {
    uint32_t steps    = durationMs / 20;   // update every 20 ms (50 Hz)
    float    stepSize = (endPct - startPct) / (float)steps;
    float    current  = startPct;

    for (uint32_t i = 0; i <= steps; i++) {
        setAllThrottlePct(current);
        Serial.printf("[RAMP] %.1f%%\n", current);
        current += stepSize;
        delay(20);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// IMU HELPERS
// ═══════════════════════════════════════════════════════════════════════════

void applyCalibration(float& gx, float& gy, float& gz,
                      float& ax, float& ay, float& az) {
    gx -= GYRO_BIAS_X; gy -= GYRO_BIAS_Y; gz -= GYRO_BIAS_Z;
    ax -= ACCEL_BIAS_X; ay -= ACCEL_BIAS_Y; az -= ACCEL_BIAS_Z;
}

bool tryInitIMU() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    for (uint8_t i = 0; i < 3; i++) {
        if (mpu.setup(MPU_ADDR)) return true;
        Serial.printf("# IMU init attempt %d/3 failed\n", i + 1);
        delay(200);
    }
    return false;
}

void warmupMadgwick() {
    Serial.println("# Warming up Madgwick filter (2 s)...");
    uint32_t end = millis() + 2000;
    while (millis() < end) {
        if (mpu.update()) {
            float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
            float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
            applyCalibration(gx, gy, gz, ax, ay, az);
            madgwick.updateIMU(gx, gy, gz, ax, ay, az);
        }
        delay(10);
    }
    Serial.printf("# Filter ready. Roll=%.1f Pitch=%.1f Yaw=%.1f\n",
        madgwick.getRoll(), madgwick.getPitch(), madgwick.getYaw());
}

// ═══════════════════════════════════════════════════════════════════════════
// TIMER ISR
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR onSampleTimer() {
    sampleReady = true;
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== NixiSat Demo Controller ===");

    // ── ESC init — identical to propeller_testing.cpp ───────────────────────
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledcSetup(escChannels[i], PWM_FREQ, PWM_RES);
        ledcAttachPin(escPins[i], escChannels[i]);
    }

    Serial.println("# Arming ESC — holding min throttle for 5 s...");
    setAllMotors(PULSE_MIN);
    delay(5000);
    Serial.println("# ESC armed.");

    // ── IMU detection ────────────────────────────────────────────────────────
    Serial.println("# Checking for IMU...");
    imuAvailable = tryInitIMU();

    if (imuAvailable) {
        Serial.println("# IMU detected: MPU9250 OK");
        madgwick.begin(SAMPLE_RATE_HZ);
        warmupMadgwick();
    } else {
        Serial.println("# IMU NOT detected — will run throttle sweep only");
    }

    // ── Mode selection ───────────────────────────────────────────────────────
    if (FORCE_MODE != MODE_AUTO) {
        activeMode = FORCE_MODE;
        Serial.printf("# Mode FORCED to %s\n",
            activeMode == MODE_STABILIZED ? "MODE_STABILIZED" : "MODE_NO_IMU");

        // Safety check: can't force stabilized without IMU
        if (activeMode == MODE_STABILIZED && !imuAvailable) {
            Serial.println("# ERROR: MODE_STABILIZED forced but IMU not found.");
            Serial.println("# Falling back to MODE_NO_IMU.");
            activeMode = MODE_NO_IMU;
        }
    } else {
        activeMode = imuAvailable ? MODE_STABILIZED : MODE_NO_IMU;
        Serial.printf("# Mode AUTO-selected: %s\n",
            activeMode == MODE_STABILIZED ? "MODE_STABILIZED" : "MODE_NO_IMU");
    }

    // ── Mode-specific startup ────────────────────────────────────────────────
    if (activeMode == MODE_STABILIZED) {
        Serial.println("# Ramping to demo throttle...");
        rampThrottle(0.0f, DEMO_HIGH_PCT, DEMO_RAMP_UP_MS);
        Serial.printf("# Holding %.0f%% for %d s — tilt cansat to see stabilization\n",
            DEMO_HIGH_PCT, DEMO_HOLD_HIGH_MS / 1000);

        // Start the 100 Hz control loop timer
        sampleTimer = timerBegin(0, 80, true);
        timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
        timerAlarmWrite(sampleTimer, SAMPLE_INTERVAL_US, true);
        timerAlarmEnable(sampleTimer);

        // Hold at DEMO_HIGH_PCT while stabilization runs (timer-driven in loop())
        // The rampThrottle already set all motors to DEMO_HIGH_PCT.
        // loop() will apply PID corrections on top of this base from now on.
        // After DEMO_HOLD_HIGH_MS the loop ramps down — tracked with demoStartMs.

    } else {
        // MODE_NO_IMU: blocking sweep, no timer needed
        Serial.println("# Running throttle sweep (no IMU)...");
        // 0% → 50% → 0% over SWEEP_DURATION_MS
        rampThrottle(0.0f,          SWEEP_PEAK_PCT, SWEEP_DURATION_MS / 2);
        rampThrottle(SWEEP_PEAK_PCT, 0.0f,          SWEEP_DURATION_MS / 2);
        setAllMotors(PULSE_MIN);
        Serial.println("# Sweep complete. Motors stopped.");
        Serial.println("# Done. Reset ESP32 to run again.");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP — only runs meaningful code in MODE_STABILIZED
// ═══════════════════════════════════════════════════════════════════════════

// Tracks when the stabilized demo phase started (set after ramp-up completes)
// We use a static flag so the ramp-down only happens once.
static bool     rampedDown    = false;
static uint32_t holdStartMs   = 0;
static bool     holdStartSet  = false;

void loop() {
    // MODE_NO_IMU is fully handled in setup() — nothing to do here
    if (activeMode != MODE_STABILIZED) return;

    // Record when the hold phase started
    if (!holdStartSet) {
        holdStartMs  = millis();
        holdStartSet = true;
    }

    // After hold time, ramp down once and stop the control loop
    if (!rampedDown && (millis() - holdStartMs > DEMO_HOLD_HIGH_MS)) {
        timerAlarmDisable(sampleTimer);   // stop the ISR
        Serial.println("# Demo hold complete — ramping down...");
        rampThrottle(DEMO_HIGH_PCT, DEMO_HOLD_PCT, DEMO_RAMP_DOWN_MS);
        Serial.printf("# Holding at %.0f%%. Reset ESP32 to run again.\n", DEMO_HOLD_PCT);
        rampedDown = true;
        return;
    }

    if (rampedDown) return;   // just hold at DEMO_HOLD_PCT, no more PID

    // ── 100 Hz stabilization loop ────────────────────────────────────────────
    if (!sampleReady) return;
    sampleReady = false;

    if (!mpu.update()) return;

    float ax = mpu.getAccX(), ay = mpu.getAccY(), az = mpu.getAccZ();
    float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
    applyCalibration(gx, gy, gz, ax, ay, az);

    madgwick.updateIMU(gx, gy, gz, ax, ay, az);

    float roll  = madgwick.getRoll();
    float pitch = madgwick.getPitch();
    float yaw   = madgwick.getYaw();

    // If tilted past safe angle, zero PID output but keep base throttle running
    // (hand-held demo — don't cut motors, just don't fight the person holding it)
    float rollOut = 0.0f, pitchOut = 0.0f, yawOut = 0.0f;
    if (fabsf(roll) < MAX_SAFE_ANGLE_DEG && fabsf(pitch) < MAX_SAFE_ANGLE_DEG) {
        rollOut  = pidRoll.update (0.0f, roll,  gx, DT);
        pitchOut = pidPitch.update(0.0f, pitch, gy, DT);
        yawOut   = pidYaw.update  (0.0f, yaw,   gz, DT);
    } else {
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
    }

    // Base throttle in µs
    int base = pctToPulse(DEMO_HIGH_PCT);

    // Mixer — standard X-frame
    int m1 = constrain(base + (int)pitchOut + (int)rollOut  - (int)yawOut, PULSE_MIN, PULSE_MAX);
    int m2 = constrain(base + (int)pitchOut - (int)rollOut  + (int)yawOut, PULSE_MIN, PULSE_MAX);
    int m3 = constrain(base - (int)pitchOut - (int)rollOut  - (int)yawOut, PULSE_MIN, PULSE_MAX);
    int m4 = constrain(base - (int)pitchOut + (int)rollOut  + (int)yawOut, PULSE_MIN, PULSE_MAX);

    setMotorPulse(escChannels[0], m1);
    setMotorPulse(escChannels[1], m2);
    setMotorPulse(escChannels[2], m3);
    setMotorPulse(escChannels[3], m4);

    // Serial output every 100 ms (10 Hz) to avoid flooding serial during demo
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.printf("[IMU] R=%.1f P=%.1f Y=%.1f | PID R=%.0f P=%.0f Y=%.0f | M=%d %d %d %d\n",
            roll, pitch, yaw,
            rollOut, pitchOut, yawOut,
            m1, m2, m3, m4);
    }
}