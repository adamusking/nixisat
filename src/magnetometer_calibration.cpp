/**
 * MPU9250 I2C test — ESP32-S3 DevKitC-1
 * Outputs CSV over Serial for live plotting with the companion Python script.
 *
 * Heading notes:
 *   - Magnetometer heading uses atan2(magY, magX). Valid as long as the board
 *     is approximately horizontal (X and Y axes in the horizontal plane).
 *     If the board is tilted, this formula is wrong — tilt compensation using
 *     accel pitch/roll is required. For bench testing flat on a table, this
 *     is fine. The zero-degree reference is wherever the mag X axis physically
 *     points when you power on; only relative changes are meaningful without
 *     a known fixed reference direction.
 *
 *   - Gyroscope heading integrates gyroZ (the yaw-rate axis when flat).
 *     It will drift over time — that is the whole point of the comparison.
 *
 * Wiring (3.3 V only):
 *   VCC -> 3.3 V | GND -> GND | SDA -> GPIO 8 | SCL -> GPIO 9
 *   AD0 -> GND  (I2C address 0x68)
 *   NCS -> leave unconnected (internal pull-up keeps device in I2C mode)
 *
 * CSV header (sent once at boot):
 *   ts_ms, aX, aY, aZ, gX, gY, gZ, mX, mY, mZ, hdgMag, hdgGyro
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

// -- I2C config ---------------------------------------------------------------
#define I2C_SDA   8
#define I2C_SCL   9
#define MPU_ADDR  0x68

// -- Sample / print rate ------------------------------------------------------
#define SAMPLE_INTERVAL_MS  20   // 50 Hz -- enough for smooth heading plots

// -----------------------------------------------------------------------------

MPU9250 mpu;

float    headingGyro  = 0.0f;
uint32_t lastSampleUs = 0;

// Normalise angle to [0, 360)
float wrap360(float deg) {
    deg = fmod(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

void halt(const char* msg) {
    while (true) { Serial.println(msg); delay(1000); }
}

// -- setup --------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("# MPU9250 CSV stream -- starting...");

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    if (!mpu.setup(MPU_ADDR)) {
        halt("ERROR: MPU9250 not found. Check wiring.");
    }

    // -- Magnetometer hard iron calibration -----------------------------------
    // Rotate the sensor slowly through ALL orientations for the full 15 s:
    //   flat, tilted forward/back/left/right, upside down, spin each axis.
    // The library samples min/max on each axis to find the center offset.
    Serial.println("# MAG CAL -- rotate sensor in all orientations for 15 s ...");
    Serial.println("# Starting in 3 s ...");
    delay(3000);
    mpu.calibrateMag();
    Serial.printf("# Mag cal done. Offsets: X=%.1f  Y=%.1f  Z=%.1f uT\n",
                  mpu.getMagBiasX(), mpu.getMagBiasY(), mpu.getMagBiasZ());
    Serial.printf("# Scale:          X=%.3f  Y=%.3f  Z=%.3f\n",
                  mpu.getMagScaleX(), mpu.getMagScaleY(), mpu.getMagScaleZ());

    // CSV header -- the Python script looks for this exact line
    Serial.println("ts_ms,aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,hdgMag,hdgGyro");

    lastSampleUs = micros();
}

// -- loop ---------------------------------------------------------------------
void loop() {
    static uint32_t lastPrint = 0;

    if (!mpu.update()) return;

    uint32_t now = millis();
    if (now - lastPrint < SAMPLE_INTERVAL_MS) return;
    lastPrint = now;

    // dt for gyro integration (microseconds for precision)
    uint32_t nowUs = micros();
    float dt = (nowUs - lastSampleUs) * 1e-6f;
    lastSampleUs = nowUs;

    // Gyro heading -- integrate Z-axis yaw rate (deg/s -> deg)
    // Positive gyroZ = counter-clockwise from above (right-hand rule, Z up).
    // Negate gyroZ if you prefer clockwise = positive.
    headingGyro = wrap360(headingGyro + mpu.getGyroZ() * dt);

    // Mag heading -- atan2 in the horizontal plane
    // Valid only when the board is flat (X and Y horizontal).
    // Negating magY converts to clockwise-positive compass convention.
    float headingMag = atan2f(-mpu.getMagY(), mpu.getMagX()) * 180.0f / PI;
    headingMag = wrap360(headingMag);

    // CSV output
    Serial.printf("%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n",
        now,
        mpu.getAccX(),  mpu.getAccY(),  mpu.getAccZ(),
        mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(),
        mpu.getMagX(),  mpu.getMagY(),  mpu.getMagZ(),
        headingMag,
        headingGyro
    );
}