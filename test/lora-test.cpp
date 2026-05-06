#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ── ESC / Motor Configuration ──────────────────────────────────────────────
const int NUM_MOTORS = 4;
const int escPins[NUM_MOTORS] = {1, 2, 3, 15};
const int escChannels[NUM_MOTORS] = {0, 1, 2, 3};
const int PWM_FREQ = 50;
const int PWM_RES  = 12;
const int PWM_MAX  = (1 << PWM_RES) - 1;
const int PULSE_MIN = 1000; // 0%
const int PULSE_MAX = 2000; // 100%

int currentPulse = PULSE_MIN; 

// ── LoRa Pins ────────────────────────────────────────────────────────────────
#define ss   14
#define rst  16
#define dio0 21
#define mosi 11
#define miso 13
#define sck  12

// ── BME280 ───────────────────────────────────────────────────────────────────
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

uint16_t count = 0;
SX1276 radio = new Module(ss, dio0, rst);

// ── Telemetry packet ─────────────────────────────────────────────────────────
typedef struct {
    uint16_t packetID;
    int16_t  temperature;
    uint16_t humidity;
    uint32_t pressure;
    int16_t  altitude;
    int32_t  latitude;
    int32_t  longitude;
    uint16_t throttlePct; // Using the battery field to send back throttle %
} __attribute__((packed)) TelemetryPacket;

// ── Motor Helper Functions ───────────────────────────────────────────────────
void updateMotors(int percent) {
    percent = constrain(percent, 0, 100);
    currentPulse = map(percent, 0, 100, PULSE_MIN, PULSE_MAX);
    
    // Convert microseconds to duty cycle (50Hz = 20,000us period)
    int duty = map(currentPulse, 0, 20000, 0, PWM_MAX);
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledcWrite(escChannels[i], duty);
    }

    Serial.print(F(">>> Power Set: "));
    Serial.print(percent);
    Serial.print(F("% (Pulse: "));
    Serial.print(currentPulse);
    Serial.println(F("us)"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println(F("--- Nixion CanSat Serial Control ---"));

    // 1. Initialize PWM & Arm ESCs
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledcSetup(escChannels[i], PWM_FREQ, PWM_RES);
        ledcAttachPin(escPins[i], escChannels[i]);
    }
    
    Serial.println(F("Arming ESCs... sending minimum throttle."));
    updateMotors(0); 
    delay(5000); 
    Serial.println(F("System Ready. Enter percentage 0-100:"));

    // 2. Initialize BME280
    if (!bme.begin(0x76)) Serial.println(F("[BME280] Not Found"));
    bme.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X2, Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X2);

    // 3. Initialize LoRa
    SPI.begin(sck, miso, mosi, ss);
    if (radio.begin() == RADIOLIB_ERR_NONE) {
        radio.setFrequency(869.525);
        radio.setBandwidth(500.0);
        radio.setSpreadingFactor(7);
        radio.setSyncWord(0xA5);
        radio.setOutputPower(17);
        Serial.println(F("LoRa Initialized."));
    }
}

void loop() {
    // --- 1. Check for Serial Input (Non-blocking) ---
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() > 0 && isDigit(input[0])) {
            int targetPower = input.toInt();
            updateMotors(targetPower);
        }
    }

    // --- 2. Periodic Telemetry (Every 2 seconds) ---
    static unsigned long lastTxTime = 0;
    if (millis() - lastTxTime > 2000) {
        lastTxTime = millis();

        TelemetryPacket packet;
        memset(&packet, 0, sizeof(TelemetryPacket));
        
        bool bmeOk = bme.takeForcedMeasurement();
        packet.packetID    = count++;
        packet.temperature = bmeOk ? (int16_t)(bme.readTemperature() * 100) : 0;
        packet.humidity    = bmeOk ? (uint16_t)(bme.readHumidity() * 100) : 0;
        packet.pressure    = bmeOk ? (uint32_t)(bme.readPressure() * 10) : 0;
        packet.altitude    = bmeOk ? (int16_t)(bme.readAltitude(SEALEVELPRESSURE_HPA) * 10) : 0;
        
        // Track the actual throttle state in telemetry
        packet.throttlePct = map(currentPulse, PULSE_MIN, PULSE_MAX, 0, 100);

        int state = radio.transmit((uint8_t *)&packet, sizeof(TelemetryPacket));
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.print(F("[LoRa TX] Pkt:")); Serial.print(packet.packetID);
            Serial.print(F(" | Pwr:")); Serial.print(packet.throttlePct);
            Serial.println(F("%"));
        }
    }
}