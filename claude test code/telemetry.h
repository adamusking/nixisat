/**
 * NixiSat — Telemetry Packet Definition
 *
 * Single binary struct sent over LoRa GFSK 9600. Layout is fixed-size and
 * little-endian (ESP32-S3 native), so the ground-station decoder can map
 * bytes directly to fields.
 *
 * Sizing budget: SX1276 GFSK variable-length frame max ≈ 60 B payload.
 * Current packet = 56 bytes — leaves headroom for sync/length/CRC framing.
 *
 * Field encoding rules (matches FDR specification, §5.4):
 *   - All multi-byte ints little-endian
 *   - Lat/Lon scaled ×1e7 → ~11 mm resolution (fits in int32_t, no int64 needed)
 *   - Temperature, attitude × 100 to keep two decimals
 *   - Altitude × 10 → 0.1 m resolution
 *   - Pressure × 10 → 0.1 Pa resolution
 *
 * Status flags bitfield — see TelemetryFlags below.
 */

#ifndef NIXISAT_TELEMETRY_H
#define NIXISAT_TELEMETRY_H

#include <Arduino.h>

// ── Flight state enum (mirrored in payload) ──────────────────────────────────
enum FlightState : uint8_t {
    STATE_BOOT              = 0,
    STATE_INIT              = 1,
    STATE_CALIBRATION       = 2,
    STATE_ARMED_PRE_LAUNCH  = 3,
    STATE_ASCENT            = 4,
    STATE_FREEFALL          = 5,    // released, awaiting deploy
    STATE_DEPLOYING         = 6,    // arms unfolding, motors spooling
    STATE_STABILIZING       = 7,    // PID engaged, holding attitude
    STATE_NAVIGATING        = 8,    // moving toward target coords
    STATE_LANDING           = 9,    // slow descent, low altitude
    STATE_LANDED            = 10,
    STATE_EMERGENCY         = 11    // motors cut, awaiting recovery
};

// ── Status flags bitfield ────────────────────────────────────────────────────
enum TelemetryFlags : uint16_t {
    FLAG_BME_OK         = 1 << 0,
    FLAG_IMU_OK         = 1 << 1,
    FLAG_GPS_FIX        = 1 << 2,
    FLAG_LORA_OK        = 1 << 3,
    FLAG_FREEFALL       = 1 << 4,
    FLAG_RELEASED       = 1 << 5,
    FLAG_MOTORS_ARMED   = 1 << 6,
    FLAG_TILT_WARN      = 1 << 7,
    FLAG_LOW_BATTERY    = 1 << 8,
    FLAG_TARGET_REACHED = 1 << 9,
    FLAG_LANDED         = 1 << 10,
    FLAG_CALIBRATION    = 1 << 11,
    FLAG_LINK_DOWN      = 1 << 12,
    FLAG_RESERVED_13    = 1 << 13,
    FLAG_RESERVED_14    = 1 << 14,
    FLAG_RESERVED_15    = 1 << 15
};

// ── Telemetry packet — total: 56 bytes ───────────────────────────────────────
#pragma pack(push, 1)
struct TelemetryPacket {
    // ── Header (8 B) ─────────────────────────────────────────────────────────
    uint32_t  packetID;         // [0..3]   monotonic counter
    uint32_t  timestamp_ms;     // [4..7]   millis() since boot

    // ── Environment (10 B) ───────────────────────────────────────────────────
    int16_t   temperature;      // [8..9]   °C × 100
    uint16_t  humidity;         // [10..11] % × 100
    uint32_t  pressure;         // [12..15] Pa × 10
    int16_t   altitude_rel;     // [16..17] metres × 10 (relative to launch)

    // ── GPS (12 B) ───────────────────────────────────────────────────────────
    int32_t   latitude;         // [18..21] degrees × 1e7
    int32_t   longitude;        // [22..25] degrees × 1e7
    int16_t   gps_altitude;     // [26..27] metres × 10
    uint16_t  gps_speed;        // [28..29] m/s × 100

    // ── Attitude (6 B) ───────────────────────────────────────────────────────
    int16_t   roll;             // [30..31] degrees × 100
    int16_t   pitch;            // [32..33] degrees × 100
    int16_t   yaw;              // [34..35] degrees × 100

    // ── Motion (6 B) ─────────────────────────────────────────────────────────
    int16_t   vertical_speed;   // [36..37] m/s × 100 (negative = descending)
    int16_t   accel_total;      // [38..39] g × 100 (vector magnitude)
    int16_t   accel_z;          // [40..41] g × 100 (Z body frame)

    // ── Motors / control (6 B) ───────────────────────────────────────────────
    uint8_t   motor_pct[4];     // [42..45] each 0..100 %
    uint8_t   battery_pct;      // [46]     0..100 %
    uint8_t   gps_satellites;   // [47]     visible satellites

    // ── Status / debug (8 B) ─────────────────────────────────────────────────
    uint8_t   flight_state;     // [48]     FlightState enum
    uint8_t   nav_distance_m;   // [49]     metres to target (capped 255)
    uint16_t  status_flags;     // [50..51] TelemetryFlags bitfield
    uint16_t  battery_mv;       // [52..53] battery voltage in mV
    uint8_t   loop_time_us;     // [54]     mean control loop time (µs / 4)
    uint8_t   reserved;         // [55]     pad — future use
};
#pragma pack(pop)

// Compile-time size check — fail loudly if struct layout drifts past LoRa max
static_assert(sizeof(TelemetryPacket) == 56,
              "TelemetryPacket must remain 56 bytes; ground decoder depends on it");

// ── Uplink command packet (ground → satellite) ──────────────────────────────
// Used to receive new target coordinates or commands during flight.
enum UplinkCommand : uint8_t {
    CMD_NONE            = 0,
    CMD_SET_TARGET      = 1,
    CMD_FORCE_LAND      = 2,
    CMD_KILL_MOTORS     = 3,
    CMD_RESUME          = 4,
    CMD_REQUEST_PHOTO   = 5
};

#pragma pack(push, 1)
struct UplinkPacket {
    uint8_t   command;          // UplinkCommand enum
    int32_t   target_lat;       // degrees × 1e7 (used by CMD_SET_TARGET)
    int32_t   target_lon;       // degrees × 1e7
    uint16_t  param;            // command-specific
    uint8_t   checksum;         // simple XOR of preceding bytes
};
#pragma pack(pop)

static_assert(sizeof(UplinkPacket) == 12, "UplinkPacket fixed at 12 bytes");

#endif // NIXISAT_TELEMETRY_H
