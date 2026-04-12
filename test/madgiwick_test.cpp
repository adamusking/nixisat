/**
 *addr 0x68
 *
 * Output (CSV over Serial, 100 Hz):
 *   ts_ms, roll, pitch, yaw, aX, aY, aZ, gX, gY, gZ
 *

 *
 *  6. Calibration offsets are applied manually (not via mpu.calibrateAccelGyro)
 *     so they survive power cycles. Run the CALIBRATION_MODE once, note the
 *     printed offsets, paste them into the USER CONFIG section below.
 *
 *  7. TUNING_MODE controls serial verbosity. In flight code, disable it —
 *     Serial.printf inside a 100 Hz loop adds measurable latency on ESP32.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <MadgwickAHRS.h>

// ═══════════════════════════════════════════════════════════════════════════
// USER CONFIG — edit these
// ═══════════════════════════════════════════════════════════════════════════

// Set to 1 to run gyro/accel calibration on startup (keep sensor STILL).
// Note the printed offsets, paste below, then set back to 0.
#define CALIBRATION_MODE    0

// Set to 1 for verbose CSV debug output. Set to 0 in flight code.
#define TUNING_MODE         1

// I2C pins
#define I2C_SDA             8
#define I2C_SCL             9
#define MPU_ADDR            0x68

// Sample rate — Madgwick needs to know this to scale beta correctly.
// 100 Hz is a good balance between responsiveness and ESP32 CPU load.
#define SAMPLE_RATE_HZ      100
#define SAMPLE_INTERVAL_US  (1000000 / SAMPLE_RATE_HZ)   // 10 000 µs

// Madgwick beta parameter.
// Higher = faster convergence to accel/mag reference, more noise sensitivity.
// Lower  = smoother, slower to correct gyro drift.
// Start at 0.1. If attitude drifts badly during flight, increase toward 0.2.
// If output is noisy/jittery, decrease toward 0.05.
#define MADGWICK_BETA       0.1f

// ── Gyro zero-rate offsets (deg/s) ──────────────────────────────────────────
// Run CALIBRATION_MODE=1 once with sensor perfectly still.
// Paste the printed GyroBias values here, then set CALIBRATION_MODE=0.
// These compensate for the sensor's inherent zero-rate error.
#define GYRO_BIAS_X         0.0f   // <-- replace with your measured value
#define GYRO_BIAS_Y         0.0f   // <-- replace with your measured value
#define GYRO_BIAS_Z         0.0f   // <-- replace with your measured value

// ── Accel zero-g offsets (g) ─────────────────────────────────────────────────
// Same: run calibration, paste values.
// When flat and still: aX~0, aY~0, aZ~1.0 is ideal.
#define ACCEL_BIAS_X        0.0f   // <-- replace with your measured value
#define ACCEL_BIAS_Y        0.0f   // <-- replace with your measured value
#define ACCEL_BIAS_Z        0.0f   // <-- replace with your measured value

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════════════

MPU9250  mpu;
Madgwick madgwick;

// ISR sets this flag every SAMPLE_INTERVAL_US microseconds.
// loop() checks it and does the actual work.
volatile bool sampleReady = false;

hw_timer_t* sampleTimer = nullptr;

// Attitude output — written by filter, read by PID (will be shared vars later)
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// ═══════════════════════════════════════════════════════════════════════════
// TIMER ISR — runs every 10 ms (100 Hz), sets flag only
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR onSampleTimer() {
    sampleReady = true;
}

// ═══════════════════════════════════════════════════════════════════════════
// HELPERS
// ═══════════════════════════════════════════════════════════════════════════

void halt(const char* msg) {
    while (true) {
        Serial.println(msg);
        delay(1000);
    }
}

// Blocks until MPU9250 responds on I2C, retrying up to maxRetries times.
// Safer than a single setup() call — I2C occasionally needs a moment after power-on.
bool initMPU(uint8_t addr, uint8_t maxRetries = 5) {
    for (uint8_t i = 0; i < maxRetries; i++) {
        if (mpu.setup(addr)) return true;
        Serial.printf("MPU9250 init attempt %d/%d failed, retrying...\n", i + 1, maxRetries);
        delay(200);
    }
    return false;
}

// Applies hardcoded bias offsets to raw sensor readings.
// Called after mpu.update() before feeding data to Madgwick.
void applyCalibration(float& gx, float& gy, float& gz,
                      float& ax, float& ay, float& az) {
    gx -= GYRO_BIAS_X;
    gy -= GYRO_BIAS_Y;
    gz -= GYRO_BIAS_Z;
    ax -= ACCEL_BIAS_X;
    ay -= ACCEL_BIAS_Y;
    az -= ACCEL_BIAS_Z;
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("# NixiSat IMU + Madgwick starting...");

    // ── I2C ────────────────────────────────────────────────────────────────
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);   // 400 kHz fast mode — needed at 100 Hz sample rate

    // ── MPU9250 init ────────────────────────────────────────────────────────
    if (!initMPU(MPU_ADDR)) {
        halt("FATAL: MPU9250 not found after retries. Check wiring and I2C address.");
    }
    Serial.println("# MPU9250 OK");

    // ── Optional runtime calibration ────────────────────────────────────────
    // Only runs if CALIBRATION_MODE=1. Keep sensor completely still.
    // The library averages ~1000 samples and prints the offsets.
    // These are NOT saved — note them and hardcode above.
#if CALIBRATION_MODE
    Serial.println("# CALIBRATION MODE: keep sensor perfectly still for 5 seconds...");
    delay(5000);
    mpu.calibrateAccelGyro();
    Serial.println("# Calibration complete. Note the bias values above.");
    Serial.printf("# Gyro bias  X=%.4f Y=%.4f Z=%.4f (deg/s)\n",
        mpu.getGyroBiasX(), mpu.getGyroBiasY(), mpu.getGyroBiasZ());
    Serial.printf("# Accel bias X=%.4f Y=%.4f Z=%.4f (g)\n",
        mpu.getAccBiasX(), mpu.getAccBiasY(), mpu.getAccBiasZ());
    Serial.println("# Paste these into USER CONFIG, set CALIBRATION_MODE=0, reflash.");
    halt("# Halted after calibration. Reflash with CALIBRATION_MODE=0.");
#endif

    // ── Madgwick init ───────────────────────────────────────────────────────
    // begin() sets the sample frequency so the library can scale beta correctly.
    // Must match your actual sample rate.
    madgwick.begin(SAMPLE_RATE_HZ);

    // The library doesn't expose a direct beta setter in all versions.
    // If your version has it, uncomment:
    // madgwick.setBeta(MADGWICK_BETA);
    // Otherwise, edit MADGWICK_BETA inside MadgwickAHRS.h (line: #define sampleFreq)
    // or subclass it. For now, the default beta (0.1) from begin() is correct.

    // ── Hardware timer ──────────────────────────────────────────────────────
    // Timer 0, prescaler 80 → 1 MHz tick → alarm at SAMPLE_INTERVAL_US ticks
    sampleTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
    timerAlarmWrite(sampleTimer, SAMPLE_INTERVAL_US, true);   // auto-reload
    timerAlarmEnable(sampleTimer);

    Serial.println("# Timer OK — running at " + String(SAMPLE_RATE_HZ) + " Hz");

#if TUNING_MODE
    // CSV header for Serial plotter / Python logger
    Serial.println("ts_ms,roll,pitch,yaw,aX,aY,aZ,gX,gY,gZ");
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    // Wait for the timer ISR to signal a new sample slot.
    // Everything else in loop() is skipped until it's time.
    if (!sampleReady) return;
    sampleReady = false;

    // ── Read IMU ────────────────────────────────────────────────────────────
    // mpu.update() returns false if no new data is ready from the sensor.
    // At 100 Hz loop rate and default MPU9250 output rate (also ≥100 Hz), 
    // this should almost never be false. If it is, we skip this cycle rather
    // than feeding stale data to the filter.
    if (!mpu.update()) return;

    // ── Get raw values ──────────────────────────────────────────────────────
    // Units: accel in g, gyro in deg/s — both correct for Madgwick directly.
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();

    // ── Apply bias corrections ──────────────────────────────────────────────
    applyCalibration(gx, gy, gz, ax, ay, az);

    // ── Madgwick update ─────────────────────────────────────────────────────
    // updateIMU() uses only accel + gyro (no magnetometer).
    // Argument order: gyro(deg/s) first, then accel(g).
    // This is the x-io convention — double-check if you switch libraries.
    madgwick.updateIMU(gx, gy, gz, ax, ay, az);

    // ── Read attitude ───────────────────────────────────────────────────────
    // Angles in degrees. Roll/pitch are accurate. Yaw drifts without mag — acceptable.
    roll  = madgwick.getRoll();
    pitch = madgwick.getPitch();
    yaw   = madgwick.getYaw();

    // ── Serial output (TUNING_MODE only) ────────────────────────────────────
    // In flight code set TUNING_MODE=0 — printf at 100 Hz adds ~0.5 ms latency.
#if TUNING_MODE
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n",
        millis(),
        roll, pitch, yaw,
        ax, ay, az,
        gx, gy, gz
    );
#endif

    // ── PID goes here ───────────────────────────────────────────────────────
    // In the next step you will call your PID update here, passing roll/pitch/yaw
    // and the raw gyro rates (gx/gy/gz) as the derivative term directly.
    // Example (not implemented yet):
    //   pidRoll.update(0.0f, roll, gx);
    //   pidPitch.update(0.0f, pitch, gy);
    //   motorMixer(throttle, pidRoll.output, pidPitch.output, pidYaw.output);
}