/**
 * NixiSat — Flight Controller Configuration
 * ESP32-S3-WROOM-1U (N16R8) — Upper MCU
 *
 * Single source of truth for ALL tunable parameters: pins, gains, thresholds.
 * Change values here, never inside main.cpp.
 *
 * SECTIONS:
 *   1. Operating mode flags
 *   2. Pin mapping
 *   3. Communication / radio
 *   4. Sensor / IMU
 *   5. Flight envelope (altitude, descent, deployment)
 *   6. PID gains
 *   7. Safety thresholds
 *   8. Target navigation (placeholder coords — update before flight)
 */

#ifndef NIXISAT_CONFIG_H
#define NIXISAT_CONFIG_H

// ═══════════════════════════════════════════════════════════════════════════
// 1. OPERATING MODE
// ═══════════════════════════════════════════════════════════════════════════

// Set to 1 for ground calibration test: ramp up, hover, ramp down. Skips
// freefall detection and flight state machine.
#define CALIBRATION_MODE        0

// Verbose serial debugging. Disable to reduce CPU load during real flight.
#define DEBUG_SERIAL            1

// ═══════════════════════════════════════════════════════════════════════════
// 2. PIN MAPPING — ESP32-S3-WROOM-1U (N16R8)
// ═══════════════════════════════════════════════════════════════════════════

// ── I2C bus (BME280 + MPU9250) ──────────────────────────────────────────────
#define PIN_I2C_SDA             8
#define PIN_I2C_SCL             9
#define I2C_FREQ_HZ             400000

#define BME280_ADDR             0x76
#define MPU9250_ADDR            0x68

// ── LoRa RA-01h (SX1276) — SPI ──────────────────────────────────────────────
#define PIN_LORA_CS             14
#define PIN_LORA_RST            16
#define PIN_LORA_DIO0           21
#define PIN_LORA_SCK            12
#define PIN_LORA_MISO           13
#define PIN_LORA_MOSI           11

// ── GPS NEO-M8M — UART1 ─────────────────────────────────────────────────────
#define PIN_GPS_RX              18      // ESP RX <- GPS TX
#define PIN_GPS_TX              17      // ESP TX -> GPS RX
#define GPS_BAUD                9600

// ── ESCs / Motors (4x, X-frame) ─────────────────────────────────────────────
//
//   Motor layout viewed from above (props down):
//
//       M1 (CW)  ╲   ╱  M2 (CCW)
//                  ╳
//       M4 (CCW) ╱   ╲  M3 (CW)
//
#define PIN_MOTOR_1             1
#define PIN_MOTOR_2             2
#define PIN_MOTOR_3             3
#define PIN_MOTOR_4             15

#define MOTOR_PWM_FREQ_HZ       50
#define MOTOR_PWM_RES_BITS      12
#define MOTOR_PULSE_MIN_US      1000    // 0% throttle / arming pulse
#define MOTOR_PULSE_MAX_US      2000    // 100% throttle

// ── UART2 link to bottom ESP32 (image-tagging / camera MCU) ─────────────────
// Avoid GPIO 43/44 — those are USB-Serial pins on ESP32-S3 and clash with the
// serial monitor. GPIO 5/6 are clean general-purpose pins.
#define PIN_LINK_TX             47       // -> bottom ESP RX
#define PIN_LINK_RX             48       // <- bottom ESP TX
#define LINK_BAUD               115200

// ── Release pin (cuts the burn wire / triggers arm deployment) ──────────────
#define PIN_RELEASE             38
#define RELEASE_ACTIVE_HIGH     1       // 1 = HIGH releases, 0 = LOW releases
#define RELEASE_HOLD_MS         1500    // how long to keep the pin asserted

// ── Battery sense (optional ADC) ────────────────────────────────────────────
#define PIN_VBAT_SENSE          5
#define VBAT_DIVIDER_RATIO      5.7f    // (R1+R2)/R2 — adjust per actual divider
#define VBAT_ADC_REF_MV         3300.0f
#define VBAT_ADC_RESOLUTION     4095.0f

// ═══════════════════════════════════════════════════════════════════════════
// 3. COMMUNICATION / RADIO
// ═══════════════════════════════════════════════════════════════════════════

// LoRa GFSK 9600 — matches FDR spec & ground station config
#define LORA_FREQ_MHZ           868.0f
#define LORA_BITRATE_KBPS       9.6f
#define LORA_FREQ_DEV_KHZ       25.0f
#define LORA_RX_BANDWIDTH_KHZ   62.5f
#define LORA_TX_POWER_DBM       17      // ~50 mW
#define LORA_SYNC_WORD          0xD391  // 2-byte sync used by ground station

// Telemetry transmit cadence (matches FDR spec: every 5 s on the GFSK channel,
// every 1 s during active flight phases)
#define TELEM_TX_INTERVAL_MS    1000
#define TELEM_TX_INTERVAL_IDLE_MS 5000

// ═══════════════════════════════════════════════════════════════════════════
// 4. SENSORS / IMU
// ═══════════════════════════════════════════════════════════════════════════

// Madgwick filter: 0.1f balanced, lower trusts gyro more, higher trusts accel
#define MADGWICK_BETA           0.1f
#define IMU_SAMPLE_RATE_HZ      200
#define CONTROL_LOOP_RATE_HZ    1000    // PID loop frequency
#define MOTOR_UPDATE_RATE_HZ    200     // motor command update rate

// ── IMU calibration biases — paste from calibration tool ────────────────────
// Use the imu+madgwick-calibration.cpp sketch from /test, then paste here.
#define GYRO_BIAS_X             0.0f
#define GYRO_BIAS_Y             0.0f
#define GYRO_BIAS_Z             0.0f
#define ACCEL_BIAS_X            0.0f
#define ACCEL_BIAS_Y            0.0f
#define ACCEL_BIAS_Z            0.0f

// BME280 sea-level reference for altitude calculation. The flight controller
// uses RELATIVE altitude (above launch point), so this is only used for
// absolute-altitude debug telemetry.
#define SEA_LEVEL_PRESSURE_HPA  1013.25f

// Altitude smoothing — exponential moving average alpha
// 0.0 = no update, 1.0 = no smoothing. 0.2 is a reasonable start.
#define ALTITUDE_FILTER_ALPHA   0.2f

// ═══════════════════════════════════════════════════════════════════════════
// 5. FLIGHT ENVELOPE
// ═══════════════════════════════════════════════════════════════════════════

// ── Deployment trigger ──────────────────────────────────────────────────────
// Both must be true: relative altitude AND freefall detected.
#define DEPLOY_MIN_ALTITUDE_M   500.0f

// Freefall = total accel magnitude below this for at least FF_HOLD_MS
#define FREEFALL_THRESHOLD_G    0.3f
#define FREEFALL_HOLD_MS        100

// Time after release pin fires before motors spool up & PID engages
#define POST_RELEASE_DELAY_MS   500

// ── Descent profile ─────────────────────────────────────────────────────────
#define TARGET_DESCENT_RATE_MPS 7.0f    // m/s downward — within 5–12 m/s spec
#define LANDING_DESCENT_RATE_MPS 2.0f   // slow approach below LANDING_ALT_M
#define LANDING_SLOWDOWN_ALT_M  20.0f   // start slowing below this AGL

// Climb-rate (vertical speed) PID tuning
// Output: throttle adjustment around HOVER_BASE_THROTTLE_PCT
#define VS_KP                   2.0f
#define VS_KI                   0.5f
#define VS_KD                   0.1f
#define VS_OUTPUT_LIMIT_PCT     30.0f   // ± from hover base

#define HOVER_BASE_THROTTLE_PCT 50.0f   // approximate hover for 4× SpeedX2 1505

// ── Landing detection ───────────────────────────────────────────────────────
// If altitude change < this for LAND_DETECT_MS at low throttle → assume landed
#define LAND_DETECT_DELTA_M     0.5f
#define LAND_DETECT_MS          2000
#define LAND_DETECT_MAX_THR_PCT 25.0f

// ═══════════════════════════════════════════════════════════════════════════
// 6. PID GAINS — attitude stabilization
// ═══════════════════════════════════════════════════════════════════════════

// Roll / Pitch — units: PWM µs correction per degree of error
#define ROLL_KP                 2.5f
#define ROLL_KI                 0.05f
#define ROLL_KD                 0.15f

#define PITCH_KP                2.5f
#define PITCH_KI                0.05f
#define PITCH_KD                0.15f

// Yaw — usually low-gain, P-only is often enough for a CanSat
#define YAW_KP                  1.5f
#define YAW_KI                  0.0f
#define YAW_KD                  0.05f

#define PID_OUTPUT_LIMIT_US     300     // max correction per axis (µs)
#define PID_INTEGRAL_LIMIT      150     // anti-windup clamp

// ═══════════════════════════════════════════════════════════════════════════
// 7. SAFETY
// ═══════════════════════════════════════════════════════════════════════════

// Cut motors if absolute roll OR pitch exceeds this
#define MAX_TILT_ANGLE_DEG      80.0f

// Cut motors if battery drops below this (if VBAT sense available)
#define MIN_BATTERY_VOLTS       12.0f   // 4S LiPo cutoff (~3.0 V/cell)

// Hard upper limit to flight time — sanity guard
#define MAX_FLIGHT_DURATION_MS  300000  // 5 minutes

// Watchdog — if the main loop doesn't tick for this long, panic-stop motors
#define LOOP_WATCHDOG_MS        500

// ═══════════════════════════════════════════════════════════════════════════
// 8. NAVIGATION TARGET (PLACEHOLDER — UPDATE BEFORE FLIGHT)
// ═══════════════════════════════════════════════════════════════════════════

// These coordinates are placeholders. The ground station can also push new
// target coordinates via LoRa uplink during flight (see handleUplink()).
#define TARGET_LATITUDE_DEG     48.143905f      // Bratislava placeholder
#define TARGET_LONGITUDE_DEG    17.108501f
#define TARGET_REACHED_RADIUS_M 10.0f           // "arrived" when within this

// Maximum horizontal lean angle while navigating (limits crosstrack tilt)
#define NAV_MAX_LEAN_ANGLE_DEG  20.0f

// Navigation P-gain — degrees of lean per metre of horizontal error
#define NAV_KP                  0.5f

#endif // NIXISAT_CONFIG_H
