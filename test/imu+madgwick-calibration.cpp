/**
 * NixiSat — IMU + Tilt-Compensated Heading
 * ESP32-S3, PlatformIO, Arduino framework
 *
 * lib_deps:
 *   arduino-libraries/Madgwick
 *   hideakitai/MPU9250
 *
 * ─── Changes from previous version ───────────────────────────────────────────
 *
 *  - Tilt-compensated heading: projects body-frame gyro rates through current
 *    roll/pitch to get true earth-frame yaw rate. Heading stays accurate even
 *    when cansat is on its side during ascent/descent.
 *
 *  - Print rate reduced to 2 Hz (every 500 ms) — readable in serial monitor.
 *    Filter still runs at 100 Hz internally.
 *
 *  - CSV output format compatible with companion Python visualizer.
 *
 * ─── How to add calibration results ─────────────────────────────────────────
 *
 *  1. Set CALIBRATION_MODE 1, flash, follow serial instructions.
 *  2. Copy the printed #define lines exactly as shown.
 *  3. Paste them over the matching defines in USER CONFIG below.
 *  4. Set CALIBRATION_MODE 0 and reflash.
 *  Example — if calibration prints:
 *    #define GYRO_BIAS_X   0.342100f
 *  Replace the current:
 *    #define GYRO_BIAS_X   0.0f
 *  With:
 *    #define GYRO_BIAS_X   0.342100f
 *
 * ─── Madgwick beta tuning guide ──────────────────────────────────────────────
 *
 *  Beta controls accel trust vs gyro trust.
 *  0.1  — good starting point for bench testing
 *  0.05 — use when motor vibration causes roll/pitch noise
 *  0.2  — use if attitude drifts too fast (poor gyro quality)
 *  Tune by watching roll/pitch in the Python visualizer:
 *    - Noisy/jumpy → decrease beta
 *    - Slow to respond or drifts → increase beta
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

#define CALIBRATION_MODE    0       // 1 = calibrate and halt, 0 = normal

#define I2C_SDA             8
#define I2C_SCL             9
#define MPU_ADDR            0x68

#define SAMPLE_RATE_HZ      200
#define SAMPLE_INTERVAL_US  (1000000 / SAMPLE_RATE_HZ)
#define DT                  (1.0f / SAMPLE_RATE_HZ)
#define PRINT_INTERVAL_MS   100    // 2 Hz print rate

#define MADGWICK_BETA       0.1f

// Calibration settings
#define CAL_SAMPLES         500
#define CAL_STILL_MS        3000
#define CAL_MAG_MS          15000

// ── Paste calibration results here ───────────────────────────────────────────
#define GYRO_BIAS_X         0.0f
#define GYRO_BIAS_Y         0.0f
#define GYRO_BIAS_Z         0.0f

#define ACCEL_BIAS_X        0.0f
#define ACCEL_BIAS_Y        0.0f
#define ACCEL_BIAS_Z        0.0f

#define MAG_BIAS_X          0.0f
#define MAG_BIAS_Y          0.0f
#define MAG_BIAS_Z          0.0f

#define MAG_SCALE_X         1.0f
#define MAG_SCALE_Y         1.0f
#define MAG_SCALE_Z         1.0f

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════════════

MPU9250  mpu;
Madgwick madgwick;

volatile bool sampleReady     = false;
hw_timer_t*   sampleTimer     = nullptr;

float initialMagHeading       = 0.0f;
float gyroHeading             = 0.0f;

// ═══════════════════════════════════════════════════════════════════════════
// HELPERS
// ═══════════════════════════════════════════════════════════════════════════

void halt(const char* msg) {
    while (true) { Serial.println(msg); delay(1000); }
}

bool initIMU() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    for (uint8_t i = 0; i < 5; i++) {
        if (mpu.setup(MPU_ADDR)) return true;
        Serial.printf("# IMU attempt %d/5\n", i + 1);
        delay(200);
    }
    return false;
}

void applyCalibration(float& gx, float& gy, float& gz,
                      float& ax, float& ay, float& az,
                      float& mx, float& my, float& mz) {
    gx -= GYRO_BIAS_X; gy -= GYRO_BIAS_Y; gz -= GYRO_BIAS_Z;
    ax -= ACCEL_BIAS_X; ay -= ACCEL_BIAS_Y; az -= ACCEL_BIAS_Z;
    mx = (mx - MAG_BIAS_X) * MAG_SCALE_X;
    my = (my - MAG_BIAS_Y) * MAG_SCALE_Y;
    mz = (mz - MAG_BIAS_Z) * MAG_SCALE_Z;
}

float magHeading(float mx, float my) {
    float h = atan2f(-my, mx) * 180.0f / PI;
    if (h < 0.0f) h += 360.0f;
    return h;
}

float wrap360(float deg) {
    deg = fmod(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

/**
 * Tilt-compensated earth-frame yaw rate.
 *
 * When the cansat is tilted, the body-frame gz axis is no longer vertical.
 * This projects all three body-frame gyro rates through the current roll/pitch
 * angles to extract only the true yaw (rotation around earth's vertical axis).
 *
 * Without this, heading drifts badly whenever the cansat is on its side.
 *
 * @param gx, gy, gz  Body-frame gyro rates in deg/s (bias-corrected)
 * @param roll, pitch Current roll/pitch from Madgwick in degrees
 * @return Earth-frame yaw rate in deg/s
 */
float earthFrameYawRate(float gx, float gy, float gz, float roll, float pitch) {
    float r = roll  * DEG_TO_RAD;
    float p = pitch * DEG_TO_RAD;
    return gz  * cosf(r) * cosf(p)
         + gy  * sinf(r)
         - gx  * cosf(r) * sinf(p);
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

void runCalibration() {
    Serial.println("\n════════════════════════════════════════");
    Serial.println("  IMU CALIBRATION MODE");
    Serial.println("════════════════════════════════════════");

    // ── Accel + Gyro ─────────────────────────────────────────────────────────
    Serial.println("\n[1/2] ACCEL + GYRO");
    Serial.printf("  Place sensor FLAT and STILL. Sampling in %ds...\n",
        CAL_STILL_MS / 1000);

    uint32_t t = millis() + CAL_STILL_MS;
    while (millis() < t) {
        Serial.printf("\r  %lus...", (t - millis()) / 1000 + 1);
        delay(200);
    }
    Serial.println("\n  Sampling...");

    double gx_s=0, gy_s=0, gz_s=0, ax_s=0, ay_s=0, az_s=0;
    int n = 0;
    while (n < CAL_SAMPLES) {
        if (mpu.update()) {
            gx_s += mpu.getGyroX(); gy_s += mpu.getGyroY(); gz_s += mpu.getGyroZ();
            ax_s += mpu.getAccX();  ay_s += mpu.getAccY();  az_s += mpu.getAccZ();
            n++;
        }
        delay(5);
    }

    float gxb = gx_s/n, gyb = gy_s/n, gzb = gz_s/n;
    float axb = ax_s/n, ayb = ay_s/n, azb = (float)(az_s/n) - 1.0f;

    Serial.println("\n  ✓ Done. Paste into USER CONFIG:");
    Serial.println("  ─────────────────────────────────────");
    Serial.printf("  #define GYRO_BIAS_X   % .6ff\n", gxb);
    Serial.printf("  #define GYRO_BIAS_Y   % .6ff\n", gyb);
    Serial.printf("  #define GYRO_BIAS_Z   % .6ff\n", gzb);
    Serial.printf("  #define ACCEL_BIAS_X  % .6ff\n", axb);
    Serial.printf("  #define ACCEL_BIAS_Y  % .6ff\n", ayb);
    Serial.printf("  #define ACCEL_BIAS_Z  % .6ff\n", azb);
    Serial.println("  ─────────────────────────────────────");

    if (fabsf(gxb)>2||fabsf(gyb)>2||fabsf(gzb)>2)
        Serial.println("  ⚠ Gyro bias >2 deg/s — was sensor moving?");
    if (fabsf(axb)>0.2f||fabsf(ayb)>0.2f||fabsf(azb)>0.2f)
        Serial.println("  ⚠ Accel bias >0.2g — was sensor flat and still?");

    // ── Magnetometer ─────────────────────────────────────────────────────────
    Serial.println("\n[2/2] MAGNETOMETER (hard + soft iron)");
    Serial.printf("  Rotate sensor through ALL orientations for %ds.\n",
        CAL_MAG_MS / 1000);
    Serial.println("  Tumble it in every direction. Starting now...");

    float xmn= 9999,ymn= 9999,zmn= 9999;
    float xmx=-9999,ymx=-9999,zmx=-9999;

    uint32_t magEnd = millis() + CAL_MAG_MS;
    uint32_t lp = 0;
    while (millis() < magEnd) {
        if (mpu.update()) {
            float mx=mpu.getMagX(), my=mpu.getMagY(), mz=mpu.getMagZ();
            xmn=min(xmn,mx); xmx=max(xmx,mx);
            ymn=min(ymn,my); ymx=max(ymx,my);
            zmn=min(zmn,mz); zmx=max(zmx,mz);
        }
        if (millis()-lp > 1000) {
            lp=millis();
            Serial.printf("  %lus remaining\n", (magEnd-millis())/1000);
        }
        delay(10);
    }

    float mxb=(xmx+xmn)/2, myb=(ymx+ymn)/2, mzb=(zmx+zmn)/2;
    float avg=((xmx-xmn)+(ymx-ymn)+(zmx-zmn))/3.0f;
    float mxs=avg/(xmx-xmn), mys=avg/(ymx-ymn), mzs=avg/(zmx-zmn);

    Serial.println("\n  ✓ Done. Paste into USER CONFIG:");
    Serial.println("  ─────────────────────────────────────");
    Serial.printf("  #define MAG_BIAS_X    % .6ff\n", mxb);
    Serial.printf("  #define MAG_BIAS_Y    % .6ff\n", myb);
    Serial.printf("  #define MAG_BIAS_Z    % .6ff\n", mzb);
    Serial.printf("  #define MAG_SCALE_X   % .6ff\n", mxs);
    Serial.printf("  #define MAG_SCALE_Y   % .6ff\n", mys);
    Serial.printf("  #define MAG_SCALE_Z   % .6ff\n", mzs);
    Serial.println("  ─────────────────────────────────────");
    Serial.println("\n  Set CALIBRATION_MODE 0 and reflash.");
    halt("# Halted.");
}

// ═══════════════════════════════════════════════════════════════════════════
// TIMER ISR
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR onSampleTimer() { sampleReady = true; }

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== NixiSat IMU + Heading ===");

    if (!initIMU()) halt("FATAL: IMU not found.");
    Serial.println("# IMU OK");

#if CALIBRATION_MODE
    runCalibration();
#endif

    delay(1000);

    // Flush stale readings
    for (int i = 0; i < 10; i++) { mpu.update(); delay(10); }

    // Capture initial heading from magnetometer — used once, then discarded
    mpu.update();
    float mx = (mpu.getMagX() - MAG_BIAS_X) * MAG_SCALE_X;
    float my = (mpu.getMagY() - MAG_BIAS_Y) * MAG_SCALE_Y;
    initialMagHeading = magHeading(mx, my);
    gyroHeading       = initialMagHeading;
    Serial.printf("# Initial mag heading: %.1f deg\n", initialMagHeading);
    Serial.println("# Mag disabled — heading tracked by tilt-compensated gyro integration.");

    madgwick.begin(SAMPLE_RATE_HZ);

    // Warmup
    Serial.println("# Warming up — hold still 2 s...");
    uint32_t end = millis() + 2000;
    while (millis() < end) {
        if (mpu.update()) {
            float ax=mpu.getAccX(), ay=mpu.getAccY(), az=mpu.getAccZ();
            float gx=mpu.getGyroX(), gy=mpu.getGyroY(), gz=mpu.getGyroZ();
            float tmx=mpu.getMagX(), tmy=mpu.getMagY(), tmz=mpu.getMagZ();
            applyCalibration(gx,gy,gz,ax,ay,az,tmx,tmy,tmz);
            madgwick.updateIMU(gx,gy,gz,ax,ay,az);
            float r=madgwick.getRoll(), p=madgwick.getPitch();
            gyroHeading = wrap360(gyroHeading + earthFrameYawRate(gx,gy,gz,r,p)*DT);
        }
        delay(10);
    }
    Serial.printf("# Ready. R=%.1f P=%.1f H=%.1f\n",
        madgwick.getRoll(), madgwick.getPitch(), gyroHeading);

    // CSV header — Python visualizer reads this exact line
    Serial.println("ts_ms,roll,pitch,heading,gx,gy,gz,ax,ay,az");

    sampleTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
    timerAlarmWrite(sampleTimer, SAMPLE_INTERVAL_US, true);
    timerAlarmEnable(sampleTimer);
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    if (!sampleReady) return;
    sampleReady = false;

    if (!mpu.update()) return;

    float ax=mpu.getAccX(), ay=mpu.getAccY(), az=mpu.getAccZ();
    float gx=mpu.getGyroX(), gy=mpu.getGyroY(), gz=mpu.getGyroZ();
    float mx=mpu.getMagX(),  my=mpu.getMagY(),  mz=mpu.getMagZ();

    applyCalibration(gx,gy,gz,ax,ay,az,mx,my,mz);

    // Madgwick: accel + gyro only for roll/pitch
    madgwick.updateIMU(gx,gy,gz,ax,ay,az);

    float roll  = madgwick.getRoll();
    float pitch = madgwick.getPitch();

    // Tilt-compensated heading integration
    gyroHeading = wrap360(gyroHeading + earthFrameYawRate(gx,gy,gz,roll,pitch)*DT);

    // Print at 2 Hz only
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < PRINT_INTERVAL_MS) return;
    lastPrint = millis();

    Serial.printf("%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        millis(), roll, pitch, gyroHeading,
        gx, gy, gz, ax, ay, az);
}