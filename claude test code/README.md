# NixiSat — Flight Controller Firmware

Firmware for the **upper ESP32-S3-WROOM-1U** flight computer on NixiSat.
Cansat Slovensko 2026, Team Nixion.

## File Layout

```
nixisat-fc/
├── platformio.ini          # PlatformIO build config
├── include/
│   ├── config.h            # ALL tunable constants — pins, gains, thresholds
│   ├── telemetry.h         # LoRa packet struct + flight state enum
│   └── pid.h               # Reusable PID controller
└── src/
    └── main.cpp            # Main flight controller — single-translation-unit
```

## Tuning Workflow

Every parameter you might want to change at the bench is in `include/config.h`,
grouped by section. **Do not edit `main.cpp` for tuning.** A typical session:

1. Set `CALIBRATION_MODE 1`, flash, run on the bench. Confirms ESCs, motors,
   sensors all behave.
2. Run the existing `imu+madgwick-calibration.cpp` sketch from `/test` to get
   gyro/accel biases. Paste the `#define GYRO_BIAS_*` / `ACCEL_BIAS_*` lines
   into `config.h`.
3. Set `CALIBRATION_MODE 0`. Tune PID gains starting low — `ROLL_KP 1.0`,
   `PITCH_KP 1.0`, all `_KI` and `_KD` at 0. Increase `_KP` until stable
   oscillation, then back off ~30% and add `_KD` to damp.
4. Set the real `TARGET_LATITUDE_DEG` / `TARGET_LONGITUDE_DEG` for the flight.

## Flight State Machine

```
BOOT → INIT → ARMED ──(launched)──→ ASCENT
                                     │
                                     ▼
                                  FREEFALL ──(alt>500m & freefall)──→ DEPLOYING
                                                                       │
                                                                       ▼
                                                                  STABILIZING
                                                                       │
                                                                       ▼
                                                                  NAVIGATING
                                                                       │
                                                                       ▼
                                                                    LANDING
                                                                       │
                                                                       ▼
                                                                    LANDED
```

Any state above `STABILIZING` will jump to `EMERGENCY` on tilt > 80° or
flight-time exceeding `MAX_FLIGHT_DURATION_MS`.

## LoRa Telemetry Packet — 56 bytes

Defined in `include/telemetry.h`. Layout (little-endian, packed):

| Offset | Field            | Type     | Size | Notes                    |
|-------:|------------------|----------|-----:|--------------------------|
|   0    | packetID         | uint32   | 4    | monotonic                |
|   4    | timestamp_ms     | uint32   | 4    | millis()                 |
|   8    | temperature      | int16    | 2    | °C × 100                 |
|  10    | humidity         | uint16   | 2    | % × 100                  |
|  12    | pressure         | uint32   | 4    | Pa × 10                  |
|  16    | altitude_rel     | int16    | 2    | m × 10 (vs launch)       |
|  18    | latitude         | int32    | 4    | deg × 1e7                |
|  22    | longitude        | int32    | 4    | deg × 1e7                |
|  26    | gps_altitude     | int16    | 2    | m × 10                   |
|  28    | gps_speed        | uint16   | 2    | m/s × 100                |
|  30    | roll             | int16    | 2    | deg × 100                |
|  32    | pitch            | int16    | 2    | deg × 100                |
|  34    | yaw              | int16    | 2    | deg × 100                |
|  36    | vertical_speed   | int16    | 2    | m/s × 100, +up           |
|  38    | accel_total      | int16    | 2    | g × 100                  |
|  40    | accel_z          | int16    | 2    | g × 100                  |
|  42    | motor_pct[4]     | uint8×4  | 4    | 0..100 each              |
|  46    | battery_pct      | uint8    | 1    | 0..100                   |
|  47    | gps_satellites   | uint8    | 1    | count                    |
|  48    | flight_state     | uint8    | 1    | FlightState enum         |
|  49    | nav_distance_m   | uint8    | 1    | metres to target (≤255)  |
|  50    | status_flags     | uint16   | 2    | bitfield (see below)     |
|  52    | battery_mv       | uint16   | 2    | millivolts               |
|  54    | loop_time_us     | uint8    | 1    | mean control loop µs ÷ 4 |
|  55    | reserved         | uint8    | 1    | future                   |

**Status flags** (bit positions):

| Bit | Flag                | Meaning                                    |
|----:|---------------------|--------------------------------------------|
|  0  | BME_OK              | BME280 reading valid                       |
|  1  | IMU_OK              | MPU9250 reading valid                      |
|  2  | GPS_FIX             | Have at least one valid fix                |
|  3  | LORA_OK             | Last TX succeeded                          |
|  4  | FREEFALL            | Currently in freefall                      |
|  5  | RELEASED            | Release pin has been fired                 |
|  6  | MOTORS_ARMED        | Motors are running                         |
|  7  | TILT_WARN           | Tilt > 60° (still flying)                  |
|  8  | LOW_BATTERY         | Below MIN_BATTERY_VOLTS                    |
|  9  | TARGET_REACHED      | Within TARGET_REACHED_RADIUS_M             |
| 10  | LANDED              | In STATE_LANDED                            |
| 11  | CALIBRATION         | Compiled with CALIBRATION_MODE=1           |
| 12  | LINK_DOWN           | Bottom ESP32 link not responding (future)  |
| 13–15 | reserved          |                                            |

## Uplink Packet — 12 bytes

```c
struct UplinkPacket {
    uint8_t  command;     // UplinkCommand enum
    int32_t  target_lat;  // deg × 1e7
    int32_t  target_lon;  // deg × 1e7
    uint16_t param;
    uint8_t  checksum;    // XOR of preceding bytes
};
```

(Reception path stubbed — to be wired into the radio in v2 once the GFSK
RX-while-TX-quiet timing is finalized.)

## Pin Mapping

| Function          | GPIO | Notes                              |
|-------------------|-----:|------------------------------------|
| I²C SDA           |  8   | BME280, MPU9250                    |
| I²C SCL           |  9   | BME280, MPU9250                    |
| LoRa CS           | 14   | SX1276                             |
| LoRa RST          |  4   |                                    |
| LoRa DIO0         | 21   |                                    |
| LoRa SCK          | 12   |                                    |
| LoRa MISO         | 13   |                                    |
| LoRa MOSI         | 11   |                                    |
| GPS RX (ESP←GPS)  | 18   | UART1 @ 9600                       |
| GPS TX (ESP→GPS)  | 17   | UART1                              |
| Motor 1           |  1   | ESC PWM, 50 Hz                     |
| Motor 2           |  2   |                                    |
| Motor 3           |  3   |                                    |
| Motor 4           | 15   |                                    |
| Link TX (→bottom) |  5   | UART2 @ 115200                     |
| Link RX (←bottom) |  6   | UART2                              |
| Release pin       | 10   | HIGH = release (configurable)      |
| VBAT sense        |  7   | ADC, divider 5.7×                  |

## Things to Verify Before First Flight

- [ ] IMU calibrated (GYRO/ACCEL biases pasted into `config.h`)
- [ ] PID gains tuned on a tethered hover rig
- [ ] `TARGET_LATITUDE_DEG` / `TARGET_LONGITUDE_DEG` set to actual target
- [ ] `VBAT_DIVIDER_RATIO` matches your actual divider
- [ ] `RELEASE_ACTIVE_HIGH` matches your release-circuit polarity
- [ ] `HOVER_BASE_THROTTLE_PCT` checked against actual hover throttle
- [ ] Ground station decoder updated to the 56-byte packet layout
