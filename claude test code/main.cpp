/**
 * NixiSat — Flight Controller (Upper ESP32-S3 WROOM-1U)
 *
 * Cansat Slovensko 2026 — Team Nixion
 *
 * RESPONSIBILITIES:
 *   - Read sensors: BME280 (env), MPU9250 (IMU), NEO-M8M (GPS)
 *   - Run Madgwick AHRS for attitude estimation
 *   - Detect freefall + altitude → fire deployment release pin
 *   - PID stabilization on roll/pitch/yaw
 *   - Vertical-speed PID for descent rate control
 *   - GPS-based navigation toward target coordinates
 *   - Stream telemetry over LoRa GFSK (downlink)
 *   - Receive uplink commands (target coords, abort, etc.)
 *   - Forward GPS + timestamps over UART2 to the bottom ESP32 (camera tagging)
 *
 * ARCHITECTURE:
 *   Single-core, cooperative loop — no FreeRTOS tasks. Each subsystem is a
 *   small struct in this file and is updated explicitly from loop(). This
 *   keeps timing deterministic and the call graph trivial to audit.
 *
 *   Two timers:
 *     - sampleTimer   @ IMU_SAMPLE_RATE_HZ — sets sampleReady flag for AHRS
 *     - controlTimer  @ CONTROL_LOOP_RATE_HZ — sets controlReady flag for PID
 *
 *   The state machine is the single arbiter of motor command vs. release pin.
 *
 * BUILD:
 *   PlatformIO `esp32-s3-devkitc-1` board, see platformio.ini
 *
 * AUTHOR: Generated for Team Nixion, NixiSat
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <MPU9250.h>
#include <MadgwickAHRS.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <RadioLib.h>

#include "config.h"
#include "telemetry.h"
#include "pid.h"

// ═══════════════════════════════════════════════════════════════════════════
// DERIVED CONSTANTS — do not edit, see config.h
// ═══════════════════════════════════════════════════════════════════════════

constexpr uint32_t SAMPLE_INTERVAL_US  = 1000000UL / IMU_SAMPLE_RATE_HZ;
constexpr uint32_t CONTROL_INTERVAL_US = 1000000UL / CONTROL_LOOP_RATE_HZ;
constexpr float    DT_CONTROL          = 1.0f / (float)CONTROL_LOOP_RATE_HZ;
constexpr int      PWM_MAX             = (1 << MOTOR_PWM_RES_BITS) - 1;
constexpr int      NUM_MOTORS          = 4;
constexpr int      MOTOR_PINS[NUM_MOTORS]     = { PIN_MOTOR_1, PIN_MOTOR_2,
                                                   PIN_MOTOR_3, PIN_MOTOR_4 };
constexpr int      MOTOR_CHANNELS[NUM_MOTORS] = { 0, 1, 2, 3 };

// ═══════════════════════════════════════════════════════════════════════════
// HARDWARE OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

Adafruit_BME280  bme;
MPU9250          mpu;
Madgwick         ahrs;
TinyGPSPlus      gps;
HardwareSerial   gpsSerial(1);
HardwareSerial   linkSerial(2);
SX1276           radio = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST);

// ═══════════════════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════════════════

struct SensorHealth {
    bool bme = false;
    bool imu = false;
    bool gps = false;     // any fix
    bool lora = false;
};

struct Attitude {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;     // bias-corrected gyro deg/s
    float ax = 0.0f, ay = 0.0f, az = 0.0f;     // accel g (body)
    float accel_total = 1.0f;                   // |a| in g
};

struct AltitudeState {
    float launch_pressure_pa = 101325.0f;       // captured at boot
    float baro_alt_m = 0.0f;                    // raw from BME280
    float relative_alt_m = 0.0f;                // smoothed, vs launch
    float prev_relative_alt_m = 0.0f;
    float vertical_speed = 0.0f;                // m/s, +up
    uint32_t last_update_ms = 0;
};

struct GpsState {
    double lat = 0.0;
    double lon = 0.0;
    float  altitude = 0.0f;
    float  speed_mps = 0.0f;
    uint8_t satellites = 0;
    bool   valid = false;
    uint32_t last_fix_ms = 0;
};

struct FreefallDetector {
    bool active = false;
    uint32_t entered_ms = 0;
};

struct LandingDetector {
    uint32_t low_motion_start_ms = 0;
    bool detecting = false;
};

struct FlightCtx {
    FlightState state = STATE_BOOT;
    uint32_t state_entered_ms = 0;
    uint32_t boot_ms = 0;
    uint32_t flight_start_ms = 0;
    uint32_t deploy_ms = 0;
    bool released = false;

    // Navigation target (mutable — uplink can change it)
    double target_lat = TARGET_LATITUDE_DEG;
    double target_lon = TARGET_LONGITUDE_DEG;
    bool target_reached = false;

    uint32_t loop_count = 0;
    uint32_t loop_time_us_sum = 0;
    uint32_t last_telem_tx_ms = 0;
    uint32_t packet_id = 0;
};

SensorHealth     health;
Attitude         att;
AltitudeState    alt;
GpsState         gpsState;
FreefallDetector ff;
LandingDetector  land;
FlightCtx        ctx;

// PID controllers
PIDController pidRoll  (ROLL_KP,  ROLL_KI,  ROLL_KD,  PID_OUTPUT_LIMIT_US, PID_INTEGRAL_LIMIT);
PIDController pidPitch (PITCH_KP, PITCH_KI, PITCH_KD, PID_OUTPUT_LIMIT_US, PID_INTEGRAL_LIMIT);
PIDController pidYaw   (YAW_KP,   YAW_KI,   YAW_KD,   PID_OUTPUT_LIMIT_US, PID_INTEGRAL_LIMIT);
PIDController pidVS    (VS_KP,    VS_KI,    VS_KD,    VS_OUTPUT_LIMIT_PCT, 20.0f);

// Current motor outputs in pulse µs (for telemetry)
int motorPulses[NUM_MOTORS] = { MOTOR_PULSE_MIN_US, MOTOR_PULSE_MIN_US,
                                 MOTOR_PULSE_MIN_US, MOTOR_PULSE_MIN_US };
float currentBaseThrottlePct = 0.0f;

// Timers and flags
volatile bool   sampleReady = false;
volatile bool   controlReady = false;
hw_timer_t*     sampleTimer  = nullptr;
hw_timer_t*     controlTimer = nullptr;

// ═══════════════════════════════════════════════════════════════════════════
// LOGGING
// ═══════════════════════════════════════════════════════════════════════════

#if DEBUG_SERIAL
    #define LOG(fmt, ...) Serial.printf(fmt "\n", ##__VA_ARGS__)
#else
    #define LOG(fmt, ...) do {} while (0)
#endif

const char* stateName(FlightState s) {
    switch (s) {
        case STATE_BOOT:             return "BOOT";
        case STATE_INIT:             return "INIT";
        case STATE_CALIBRATION:      return "CALIBRATION";
        case STATE_ARMED_PRE_LAUNCH: return "ARMED";
        case STATE_ASCENT:           return "ASCENT";
        case STATE_FREEFALL:         return "FREEFALL";
        case STATE_DEPLOYING:        return "DEPLOYING";
        case STATE_STABILIZING:      return "STABILIZING";
        case STATE_NAVIGATING:       return "NAVIGATING";
        case STATE_LANDING:          return "LANDING";
        case STATE_LANDED:           return "LANDED";
        case STATE_EMERGENCY:        return "EMERGENCY";
    }
    return "?";
}

void setState(FlightState newState) {
    if (newState == ctx.state) return;
    LOG("[STATE] %s -> %s @ %lu ms",
        stateName(ctx.state), stateName(newState), millis());
    ctx.state = newState;
    ctx.state_entered_ms = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════════════

inline int pulseToDuty(int pulse_us) {
    return map(pulse_us, 0, 20000, 0, PWM_MAX);
}

inline int pctToPulse(float pct) {
    pct = constrain(pct, 0.0f, 100.0f);
    return MOTOR_PULSE_MIN_US +
           (int)(pct * 0.01f * (MOTOR_PULSE_MAX_US - MOTOR_PULSE_MIN_US));
}

void writeMotor(int idx, int pulse_us) {
    pulse_us = constrain(pulse_us, MOTOR_PULSE_MIN_US, MOTOR_PULSE_MAX_US);
    motorPulses[idx] = pulse_us;
    ledcWrite(MOTOR_CHANNELS[idx], pulseToDuty(pulse_us));
}

void writeAllMotorsPulse(int pulse_us) {
    for (int i = 0; i < NUM_MOTORS; i++) writeMotor(i, pulse_us);
}

void writeAllMotorsPct(float pct) {
    int p = pctToPulse(pct);
    for (int i = 0; i < NUM_MOTORS; i++) writeMotor(i, p);
}

void killMotors() {
    writeAllMotorsPulse(MOTOR_PULSE_MIN_US);
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    pidVS.reset();
}

void initMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledcSetup(MOTOR_CHANNELS[i], MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
        ledcAttachPin(MOTOR_PINS[i], MOTOR_CHANNELS[i]);
    }
    LOG("[MOT] Arming ESCs (5 s @ min throttle)...");
    writeAllMotorsPulse(MOTOR_PULSE_MIN_US);
    delay(5000);
    LOG("[MOT] ESCs armed.");
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR INIT
// ═══════════════════════════════════════════════════════════════════════════

bool initBME280() {
    if (!bme.begin(BME280_ADDR)) {
        LOG("[BME] init failed @ 0x%02X", BME280_ADDR);
        return false;
    }
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,    // temp
                    Adafruit_BME280::SAMPLING_X16,   // pressure (more precise → better altitude)
                    Adafruit_BME280::SAMPLING_X1,    // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5);

    // Capture launch pressure as zero-altitude reference
    delay(50);
    float sum = 0.0f;
    int n = 0;
    for (int i = 0; i < 30; i++) {
        float p = bme.readPressure();
        if (p > 50000.0f && p < 110000.0f) { sum += p; n++; }
        delay(20);
    }
    alt.launch_pressure_pa = (n > 0) ? (sum / n) : 101325.0f;
    LOG("[BME] launch pressure = %.1f Pa", alt.launch_pressure_pa);
    return true;
}

bool initIMU() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_FREQ_HZ);

    for (int i = 0; i < 5; i++) {
        if (mpu.setup(MPU9250_ADDR)) {
            ahrs.begin(IMU_SAMPLE_RATE_HZ);
            LOG("[IMU] MPU9250 OK");
            return true;
        }
        LOG("[IMU] init attempt %d/5", i + 1);
        delay(200);
    }
    return false;
}

bool initLoRa() {
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

    int state = radio.beginFSK();
    if (state != RADIOLIB_ERR_NONE) {
        LOG("[LoRa] beginFSK failed: %d", state);
        return false;
    }

    state  = radio.setFrequency(LORA_FREQ_MHZ);
    state |= radio.setBitRate(LORA_BITRATE_KBPS);
    state |= radio.setFrequencyDeviation(LORA_FREQ_DEV_KHZ);
    state |= radio.setRxBandwidth(LORA_RX_BANDWIDTH_KHZ);
    state |= radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    state |= radio.setOutputPower(LORA_TX_POWER_DBM);
    state |= radio.setCRC(true);

    if (state != RADIOLIB_ERR_NONE) {
        LOG("[LoRa] config error: %d", state);
        return false;
    }

    state = radio.variablePacketLengthMode(60);
    if (state != RADIOLIB_ERR_NONE) {
        LOG("[LoRa] varlen error: %d", state);
        return false;
    }

    LOG("[LoRa] GFSK %g MHz / %g kbps / +%d dBm",
        LORA_FREQ_MHZ, LORA_BITRATE_KBPS, LORA_TX_POWER_DBM);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR UPDATE
// ═══════════════════════════════════════════════════════════════════════════

void applyImuCal(float& gx, float& gy, float& gz,
                 float& ax, float& ay, float& az) {
    gx -= GYRO_BIAS_X; gy -= GYRO_BIAS_Y; gz -= GYRO_BIAS_Z;
    ax -= ACCEL_BIAS_X; ay -= ACCEL_BIAS_Y; az -= ACCEL_BIAS_Z;
}

// Called from the IMU sample timer — updates Madgwick + computes |a|
void updateImuAndAhrs() {
    if (!health.imu) return;
    if (!mpu.update()) return;

    float ax = mpu.getAccX(),  ay = mpu.getAccY(),  az = mpu.getAccZ();
    float gx = mpu.getGyroX(), gy = mpu.getGyroY(), gz = mpu.getGyroZ();
    applyImuCal(gx, gy, gz, ax, ay, az);

    ahrs.updateIMU(gx, gy, gz, ax, ay, az);

    att.roll  = ahrs.getRoll();
    att.pitch = ahrs.getPitch();
    att.yaw   = ahrs.getYaw();
    att.gx = gx; att.gy = gy; att.gz = gz;
    att.ax = ax; att.ay = ay; att.az = az;
    att.accel_total = sqrtf(ax * ax + ay * ay + az * az);
}

void updateBaroAltitude() {
    if (!health.bme) return;

    // takeForcedMeasurement only if in MODE_FORCED — we use MODE_NORMAL, just read
    float p = bme.readPressure();
    if (p < 50000.0f || p > 110000.0f) return;   // garbage value

    // Barometric formula (relative to launch pressure)
    float ratio = p / alt.launch_pressure_pa;
    float new_relative_alt = 44330.0f * (1.0f - powf(ratio, 0.190294957f));
    alt.baro_alt_m = 44330.0f * (1.0f - powf(p / 101325.0f, 0.190294957f));

    // EMA smoothing
    alt.relative_alt_m = ALTITUDE_FILTER_ALPHA * new_relative_alt
                       + (1.0f - ALTITUDE_FILTER_ALPHA) * alt.relative_alt_m;

    // Vertical speed via finite difference
    uint32_t now = millis();
    if (alt.last_update_ms != 0) {
        float dt = (now - alt.last_update_ms) * 0.001f;
        if (dt > 0.0f) {
            alt.vertical_speed = (alt.relative_alt_m - alt.prev_relative_alt_m) / dt;
        }
    }
    alt.prev_relative_alt_m = alt.relative_alt_m;
    alt.last_update_ms = now;
}

void serviceGps() {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());

    if (gps.location.isValid() && gps.location.isUpdated()) {
        gpsState.lat = gps.location.lat();
        gpsState.lon = gps.location.lng();
        gpsState.valid = true;
        gpsState.last_fix_ms = millis();
        health.gps = true;
    }
    if (gps.altitude.isValid())   gpsState.altitude   = gps.altitude.meters();
    if (gps.speed.isValid())      gpsState.speed_mps  = gps.speed.mps();
    if (gps.satellites.isValid()) gpsState.satellites = gps.satellites.value();
}

// ═══════════════════════════════════════════════════════════════════════════
// FREEFALL DETECTION — accel magnitude < threshold for FREEFALL_HOLD_MS
// ═══════════════════════════════════════════════════════════════════════════

void updateFreefall() {
    bool low_g = att.accel_total < FREEFALL_THRESHOLD_G;
    uint32_t now = millis();

    if (low_g && !ff.active) {
        ff.entered_ms = now;
        ff.active = true;
    } else if (!low_g) {
        ff.active = false;
        ff.entered_ms = 0;
    }
}

bool isInFreefall() {
    return ff.active && (millis() - ff.entered_ms >= FREEFALL_HOLD_MS);
}

// ═══════════════════════════════════════════════════════════════════════════
// RELEASE PIN — burns the wire / drops the arms
// ═══════════════════════════════════════════════════════════════════════════

void initReleasePin() {
    pinMode(PIN_RELEASE, OUTPUT);
    digitalWrite(PIN_RELEASE, RELEASE_ACTIVE_HIGH ? LOW : HIGH);
}

void fireRelease() {
    if (ctx.released) return;
    LOG("[REL] Firing release pin (alt=%.1f m, |a|=%.2f g)",
        alt.relative_alt_m, att.accel_total);
    digitalWrite(PIN_RELEASE, RELEASE_ACTIVE_HIGH ? HIGH : LOW);
    ctx.released = true;
    ctx.deploy_ms = millis();

    // The pin stays asserted until checked in loop — we'll release after RELEASE_HOLD_MS
}

void serviceReleasePin() {
    if (ctx.released &&
        (millis() - ctx.deploy_ms > RELEASE_HOLD_MS)) {
        digitalWrite(PIN_RELEASE, RELEASE_ACTIVE_HIGH ? LOW : HIGH);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NAVIGATION — bearing & distance to target
// ═══════════════════════════════════════════════════════════════════════════

struct NavResult {
    float distance_m;
    float bearing_deg;       // 0=N, 90=E
    float north_error_m;     // signed: +ve if target north of us
    float east_error_m;      // signed: +ve if target east of us
};

NavResult computeNavigation() {
    NavResult r{0, 0, 0, 0};
    if (!gpsState.valid) return r;

    constexpr double EARTH_R = 6371000.0;
    double lat1 = gpsState.lat * DEG_TO_RAD;
    double lat2 = ctx.target_lat * DEG_TO_RAD;
    double dLat = (ctx.target_lat - gpsState.lat) * DEG_TO_RAD;
    double dLon = (ctx.target_lon - gpsState.lon) * DEG_TO_RAD;

    // Equirectangular approximation — accurate enough for short ranges (<10 km)
    double x = dLon * cos((lat1 + lat2) / 2.0);
    double y = dLat;
    r.distance_m   = (float)(sqrt(x * x + y * y) * EARTH_R);
    r.bearing_deg  = (float)(atan2(x, y) * RAD_TO_DEG);
    if (r.bearing_deg < 0) r.bearing_deg += 360.0f;

    r.north_error_m = (float)(y * EARTH_R);
    r.east_error_m  = (float)(x * EARTH_R);
    return r;
}

// ═══════════════════════════════════════════════════════════════════════════
// CONTROL LOOP — runs at CONTROL_LOOP_RATE_HZ
//   Combines:
//     * Vertical-speed PID (sets base throttle)
//     * Roll/pitch/yaw PID (attitude correction)
//     * Navigation (sets desired roll/pitch lean)
// ═══════════════════════════════════════════════════════════════════════════

void runStabilizationAndNavigation() {
    if (!health.imu) {
        killMotors();
        return;
    }

    // --- Safety: tilt limit ---
    if (fabsf(att.roll) > MAX_TILT_ANGLE_DEG ||
        fabsf(att.pitch) > MAX_TILT_ANGLE_DEG) {
        LOG("[SAFE] Tilt %.1f/%.1f exceeded %.0f° — emergency stop",
            att.roll, att.pitch, MAX_TILT_ANGLE_DEG);
        killMotors();
        setState(STATE_EMERGENCY);
        return;
    }

    // --- Determine setpoints ---
    float roll_sp  = 0.0f;
    float pitch_sp = 0.0f;
    float yaw_sp   = 0.0f;
    float vs_sp    = -TARGET_DESCENT_RATE_MPS;

    // Slow down for landing
    if (alt.relative_alt_m < LANDING_SLOWDOWN_ALT_M) {
        vs_sp = -LANDING_DESCENT_RATE_MPS;
    }

    // Navigation tilt — when in NAVIGATING state and have GPS
    if (ctx.state == STATE_NAVIGATING && gpsState.valid) {
        NavResult nav = computeNavigation();
        ctx.target_reached = (nav.distance_m < TARGET_REACHED_RADIUS_M);

        if (!ctx.target_reached) {
            // Convert NE error to body-frame lean. Assumes yaw is small or
            // we treat lean in earth-frame and let yaw rotate around it.
            // Pitch forward (positive) = move north, roll right (positive) = move east.
            // For simplicity (no yaw compensation in v1) we lean directly:
            float pitch_cmd = -NAV_KP * nav.north_error_m;
            float roll_cmd  =  NAV_KP * nav.east_error_m;

            pitch_sp = constrain(pitch_cmd, -NAV_MAX_LEAN_ANGLE_DEG, NAV_MAX_LEAN_ANGLE_DEG);
            roll_sp  = constrain(roll_cmd,  -NAV_MAX_LEAN_ANGLE_DEG, NAV_MAX_LEAN_ANGLE_DEG);
        }
    }

    // --- Vertical-speed PID → base throttle ---
    float vs_correction = pidVS.updateDerivOnError(vs_sp, alt.vertical_speed, DT_CONTROL);
    currentBaseThrottlePct = constrain(HOVER_BASE_THROTTLE_PCT + vs_correction, 0.0f, 100.0f);

    // --- Attitude PIDs ---
    float roll_out  = pidRoll.update (roll_sp,  att.roll,  att.gx, DT_CONTROL);
    float pitch_out = pidPitch.update(pitch_sp, att.pitch, att.gy, DT_CONTROL);
    float yaw_out   = pidYaw.update  (yaw_sp,   att.yaw,   att.gz, DT_CONTROL);

    // --- Motor mixer (X-frame) ---
    int base = pctToPulse(currentBaseThrottlePct);
    int m1 = base + (int)pitch_out + (int)roll_out  - (int)yaw_out;
    int m2 = base + (int)pitch_out - (int)roll_out  + (int)yaw_out;
    int m3 = base - (int)pitch_out - (int)roll_out  - (int)yaw_out;
    int m4 = base - (int)pitch_out + (int)roll_out  + (int)yaw_out;

    writeMotor(0, m1);
    writeMotor(1, m2);
    writeMotor(2, m3);
    writeMotor(3, m4);
}

// ═══════════════════════════════════════════════════════════════════════════
// LANDING DETECTION
// ═══════════════════════════════════════════════════════════════════════════

void checkLanding() {
    uint32_t now = millis();

    bool low_throttle = currentBaseThrottlePct < LAND_DETECT_MAX_THR_PCT;
    // Vertical speed should be ≈ 0 if we're sitting on the ground
    bool slow_alt_change = fabsf(alt.vertical_speed) < LAND_DETECT_DELTA_M;
    // also low absolute altitude — don't trigger up high
    bool near_ground = alt.relative_alt_m < LANDING_SLOWDOWN_ALT_M;

    if (low_throttle && slow_alt_change && near_ground) {
        if (!land.detecting) {
            land.detecting = true;
            land.low_motion_start_ms = now;
        } else if (now - land.low_motion_start_ms > LAND_DETECT_MS) {
            LOG("[LAND] Landing detected (alt=%.1f m, vs=%.2f m/s)",
                alt.relative_alt_m, alt.vertical_speed);
            killMotors();
            setState(STATE_LANDED);
        }
    } else {
        land.detecting = false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FLIGHT STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════

void runFlightStateMachine() {
    switch (ctx.state) {

    case STATE_ARMED_PRE_LAUNCH:
        // Waiting on launch tower / hand release. Watch for upward motion or
        // freefall — either signals we are no longer at rest.
        if (alt.relative_alt_m > 5.0f) {
            ctx.flight_start_ms = millis();
            setState(STATE_ASCENT);
        } else if (isInFreefall()) {
            // Direct hand-drop scenario
            ctx.flight_start_ms = millis();
            setState(STATE_FREEFALL);
        }
        break;

    case STATE_ASCENT:
        // Climbing in the rocket. Once we start falling, transition.
        if (isInFreefall()) {
            setState(STATE_FREEFALL);
        }
        break;

    case STATE_FREEFALL:
        // Check deployment trigger: altitude OR freefall (per requirements
        // wording, the deployment fires once altitude>500m AND freefall).
        if (alt.relative_alt_m >= DEPLOY_MIN_ALTITUDE_M && isInFreefall()) {
            fireRelease();
            setState(STATE_DEPLOYING);
        }
        // Safety override: if we're below 200m and still in freefall, deploy anyway.
        else if (alt.relative_alt_m > 200.0f && alt.relative_alt_m < 250.0f
                 && isInFreefall()) {
            LOG("[STATE] Low-altitude emergency deployment");
            fireRelease();
            setState(STATE_DEPLOYING);
        }
        break;

    case STATE_DEPLOYING:
        // Wait POST_RELEASE_DELAY_MS for arms to swing out, then engage PID.
        if (millis() - ctx.deploy_ms >= POST_RELEASE_DELAY_MS) {
            // Spool to hover throttle smoothly (PID will handle once active)
            writeAllMotorsPct(HOVER_BASE_THROTTLE_PCT * 0.6f);
            pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); pidVS.reset();
            setState(STATE_STABILIZING);
        }
        break;

    case STATE_STABILIZING:
        runStabilizationAndNavigation();
        // After 1 s of stable flight, start navigating
        if (millis() - ctx.state_entered_ms > 1000 && gpsState.valid) {
            setState(STATE_NAVIGATING);
        }
        break;

    case STATE_NAVIGATING:
        runStabilizationAndNavigation();
        // Transition to landing when low enough OR target reached
        if (alt.relative_alt_m < LANDING_SLOWDOWN_ALT_M) {
            setState(STATE_LANDING);
        }
        break;

    case STATE_LANDING:
        runStabilizationAndNavigation();
        checkLanding();
        break;

    case STATE_LANDED:
        killMotors();
        break;

    case STATE_EMERGENCY:
        killMotors();
        break;

    default:
        break;
    }

    // Universal safety: max flight duration
    if (ctx.flight_start_ms > 0 &&
        (millis() - ctx.flight_start_ms) > MAX_FLIGHT_DURATION_MS &&
        ctx.state != STATE_LANDED) {
        LOG("[SAFE] MAX_FLIGHT_DURATION exceeded");
        killMotors();
        setState(STATE_EMERGENCY);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// CALIBRATION ROUTINE — ground test only
// ═══════════════════════════════════════════════════════════════════════════

void runCalibrationSequence() {
    LOG("[CAL] Starting calibration sequence");
    setState(STATE_CALIBRATION);

    // Ramp up over 3 s
    LOG("[CAL] Ramping up...");
    for (int pct = 0; pct <= 30; pct++) {
        writeAllMotorsPct((float)pct);
        delay(100);
    }

    LOG("[CAL] Hovering for 3 s");
    delay(3000);

    LOG("[CAL] Ramping down...");
    for (int pct = 30; pct >= 0; pct--) {
        writeAllMotorsPct((float)pct);
        delay(100);
    }

    killMotors();
    LOG("[CAL] Calibration complete. Halting.");
    setState(STATE_LANDED);
}

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY VOLTAGE
// ═══════════════════════════════════════════════════════════════════════════

uint16_t readBatteryMv() {
    int adc = analogRead(PIN_VBAT_SENSE);
    float v_pin = (adc / VBAT_ADC_RESOLUTION) * VBAT_ADC_REF_MV;
    return (uint16_t)(v_pin * VBAT_DIVIDER_RATIO);
}

uint8_t batteryPct(uint16_t mv) {
    // 4S LiPo nominal 14.8 V (3.7×4), full 16.8 V, empty 12.0 V
    constexpr float V_FULL  = 16800.0f;
    constexpr float V_EMPTY = 12000.0f;
    if (mv >= V_FULL)  return 100;
    if (mv <= V_EMPTY) return 0;
    return (uint8_t)((mv - V_EMPTY) * 100.0f / (V_FULL - V_EMPTY));
}

// ═══════════════════════════════════════════════════════════════════════════
// LINK TO BOTTOM ESP32 (camera MCU) — GPS + timestamp for image tagging
// ═══════════════════════════════════════════════════════════════════════════

void sendLinkUpdate() {
    // Compact ASCII line — easy to parse on the camera side.
    // Format: $NIX,<ms>,<lat>,<lon>,<alt>,<state>*<nl>
    linkSerial.printf("$NIX,%lu,%.7f,%.7f,%.1f,%u\n",
        millis(),
        gpsState.lat,
        gpsState.lon,
        alt.relative_alt_m,
        (unsigned)ctx.state);
}

// ═══════════════════════════════════════════════════════════════════════════
// TELEMETRY PACKAGING & TX
// ═══════════════════════════════════════════════════════════════════════════

uint16_t buildStatusFlags() {
    uint16_t f = 0;
    if (health.bme)              f |= FLAG_BME_OK;
    if (health.imu)              f |= FLAG_IMU_OK;
    if (gpsState.valid)          f |= FLAG_GPS_FIX;
    if (health.lora)             f |= FLAG_LORA_OK;
    if (ff.active)               f |= FLAG_FREEFALL;
    if (ctx.released)            f |= FLAG_RELEASED;
    if (ctx.state >= STATE_DEPLOYING && ctx.state <= STATE_LANDING)
                                 f |= FLAG_MOTORS_ARMED;
    if (fabsf(att.roll) > 60.0f || fabsf(att.pitch) > 60.0f)
                                 f |= FLAG_TILT_WARN;
    if (ctx.target_reached)      f |= FLAG_TARGET_REACHED;
    if (ctx.state == STATE_LANDED) f |= FLAG_LANDED;
#if CALIBRATION_MODE
    f |= FLAG_CALIBRATION;
#endif
    return f;
}

void buildTelemetryPacket(TelemetryPacket& p) {
    memset(&p, 0, sizeof(p));

    p.packetID     = ++ctx.packet_id;
    p.timestamp_ms = millis();

    if (health.bme) {
        p.temperature  = (int16_t)(bme.readTemperature() * 100.0f);
        p.humidity     = (uint16_t)(bme.readHumidity()    * 100.0f);
        p.pressure     = (uint32_t)(bme.readPressure()    * 10.0f);
    }
    p.altitude_rel = (int16_t)(alt.relative_alt_m * 10.0f);

    p.latitude       = (int32_t)(gpsState.lat * 1e7);
    p.longitude      = (int32_t)(gpsState.lon * 1e7);
    p.gps_altitude   = (int16_t)(gpsState.altitude * 10.0f);
    p.gps_speed      = (uint16_t)(gpsState.speed_mps * 100.0f);
    p.gps_satellites = gpsState.satellites;

    p.roll  = (int16_t)(att.roll  * 100.0f);
    p.pitch = (int16_t)(att.pitch * 100.0f);
    p.yaw   = (int16_t)(att.yaw   * 100.0f);

    p.vertical_speed = (int16_t)(alt.vertical_speed * 100.0f);
    p.accel_total    = (int16_t)(att.accel_total    * 100.0f);
    p.accel_z        = (int16_t)(att.az             * 100.0f);

    for (int i = 0; i < NUM_MOTORS; i++) {
        p.motor_pct[i] = (uint8_t)map(motorPulses[i],
            MOTOR_PULSE_MIN_US, MOTOR_PULSE_MAX_US, 0, 100);
    }

    uint16_t mv = readBatteryMv();
    p.battery_mv  = mv;
    p.battery_pct = batteryPct(mv);

    p.flight_state = (uint8_t)ctx.state;
    p.status_flags = buildStatusFlags();

    NavResult nav = computeNavigation();
    p.nav_distance_m = (uint8_t)constrain(nav.distance_m, 0.0f, 255.0f);

    // Mean control loop µs / 4 (so 1023 µs fits in u8 nicely)
    if (ctx.loop_count > 0) {
        uint32_t mean = ctx.loop_time_us_sum / ctx.loop_count;
        p.loop_time_us = (uint8_t)constrain((int)(mean / 4), 0, 255);
        ctx.loop_count = 0;
        ctx.loop_time_us_sum = 0;
    }
}

void sendTelemetry() {
    TelemetryPacket p;
    buildTelemetryPacket(p);

    int state = radio.transmit((uint8_t*)&p, sizeof(TelemetryPacket));
    if (state == RADIOLIB_ERR_NONE) {
        health.lora = true;
    } else {
        LOG("[LoRa] TX err %d — re-init", state);
        health.lora = false;
        // Best-effort re-init in background; don't block the flight loop
        radio.beginFSK();
        radio.setFrequency(LORA_FREQ_MHZ);
        radio.setBitRate(LORA_BITRATE_KBPS);
        radio.setFrequencyDeviation(LORA_FREQ_DEV_KHZ);
        radio.setRxBandwidth(LORA_RX_BANDWIDTH_KHZ);
        radio.setOutputPower(LORA_TX_POWER_DBM);
        radio.setCRC(true);
        radio.variablePacketLengthMode(60);
    }

#if DEBUG_SERIAL
    LOG("[TX %lu] st=%s alt=%.1f vs=%.2f R=%.1f P=%.1f Y=%.1f gps=%d sats=%d ff=%d",
        (unsigned long)p.packetID, stateName(ctx.state),
        alt.relative_alt_m, alt.vertical_speed,
        att.roll, att.pitch, att.yaw,
        (int)gpsState.valid, (int)gpsState.satellites,
        (int)ff.active);
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// TIMER ISRs
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR onSampleTimer()  { sampleReady  = true; }
void IRAM_ATTR onControlTimer() { controlReady = true; }

void startTimers() {
    sampleTimer = timerBegin(0, 80, true);          // 1 µs tick
    timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
    timerAlarmWrite(sampleTimer, SAMPLE_INTERVAL_US, true);
    timerAlarmEnable(sampleTimer);

    controlTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(controlTimer, &onControlTimer, true);
    timerAlarmWrite(controlTimer, CONTROL_INTERVAL_US, true);
    timerAlarmEnable(controlTimer);
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);
    LOG("\n=== NixiSat Flight Controller ===");
    LOG("Build: %s %s", __DATE__, __TIME__);

    ctx.boot_ms = millis();

    // Pins
    initReleasePin();
    pinMode(PIN_VBAT_SENSE, INPUT);

    // Bottom-ESP link
    linkSerial.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);

    // GPS
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // ESCs (5 s arming delay built in)
    initMotors();

    // Sensors
    health.bme  = initBME280();
    health.imu  = initIMU();
    health.lora = initLoRa();

    setState(STATE_INIT);

#if CALIBRATION_MODE
    runCalibrationSequence();
    return;
#endif

    // Start timers BEFORE arming the flight state machine
    startTimers();

    setState(STATE_ARMED_PRE_LAUNCH);
    LOG("[INIT] Armed — waiting for launch event.");
}

// ═══════════════════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
#if CALIBRATION_MODE
    // Calibration ran in setup(); just hold here.
    delay(1000);
    return;
#endif

    uint32_t loop_start = micros();

    // 1. Sensor updates
    if (sampleReady) {
        sampleReady = false;
        updateImuAndAhrs();
    }

    // 2. Slower-cadence sensors
    static uint32_t lastBaroMs = 0;
    if (millis() - lastBaroMs >= 50) {       // 20 Hz baro
        lastBaroMs = millis();
        updateBaroAltitude();
    }

    serviceGps();                            // continuous, parses bytes as they arrive

    // 3. Freefall detection (always, regardless of state)
    updateFreefall();

    // 4. Control loop tick
    if (controlReady) {
        controlReady = false;
        runFlightStateMachine();
    }

    // 5. Release-pin auto-deassert
    serviceReleasePin();

    // 6. Periodic outputs
    uint32_t now = millis();
    uint32_t telem_interval = (ctx.state == STATE_ARMED_PRE_LAUNCH ||
                               ctx.state == STATE_LANDED)
                              ? TELEM_TX_INTERVAL_IDLE_MS
                              : TELEM_TX_INTERVAL_MS;
    if (now - ctx.last_telem_tx_ms >= telem_interval) {
        ctx.last_telem_tx_ms = now;
        sendTelemetry();
        sendLinkUpdate();
    }

    // 7. Loop-time bookkeeping for debug telemetry
    uint32_t loop_us = micros() - loop_start;
    ctx.loop_time_us_sum += loop_us;
    ctx.loop_count++;
}
