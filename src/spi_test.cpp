#include <RadioLib.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <TinyGPSPlus.h>

#define PIN_CS 14
#define PIN_DIO0 21
#define PIN_RESET 4

TinyGPSPlus gps;
HardwareSerial GPSSerial(1); // UART1, pins 18(RX)/17(TX)

SX1276 radio = new Module(PIN_CS, PIN_DIO0, PIN_RESET);
Adafruit_BME280 bme;

// Binary telemetry struct - 32bytes. Maximum 64 bajtov pre sx1276
#pragma pack(1)
struct Telemetry
{
  uint32_t packetID;   // packet counter
  int16_t temperature; // °C  × 100
  uint16_t humidity;   // %   × 100
  uint32_t pressure;   // Pa  × 10
  int16_t altitude;    // m   × 10
  int64_t latitude;    // °   × 1e8
  int64_t longitude;   // °   × 1e8
  uint16_t battery;    // mV
}; // total: 4+2+2+4+2+8+8+2 = 32 bytes
#pragma pack()

#define MAX_FRAME 60

static const uint32_t TX_INTERVAL_MS = 100;

// =============================================================================
// Radio init — GFSK9600
bool initGFSK()
{
  Serial.println("[INIT] GFSK9600 transmitter...");

  int state = radio.beginFSK();
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[FAIL] beginFSK: %d\n", state);
    return false;
  }

  state = radio.setFrequency(868.0);
  state |= radio.setBitRate(9.6);
  state |= radio.setFrequencyDeviation(25.0);
  state |= radio.setRxBandwidth(62.5);
  state |= radio.setDataShaping(RADIOLIB_SHAPING_0_5);
  state |= radio.setOutputPower(17);
  state |= radio.setCRC(true);

  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[FAIL] radio config: %d\n", state);
    return false;
  }

  state = radio.variablePacketLengthMode(MAX_FRAME);
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[FAIL] variablePacketLengthMode: %d\n", state);
    return false;
  }

  Serial.println("[OK] Radio ready");
  return true;
}

void initGFSKBlocking()
{
  while (!initGFSK())
  {
    Serial.println("[RETRY] init failed, retrying in 2s...");
    delay(2000);
  }
}




// =============================================================================
// Telemetry build and send


static void buildAndSendTelemetry()
{
  static uint32_t packetID = 0;
  packetID++;

  bme.takeForcedMeasurement();

  Telemetry t;
  t.packetID = packetID;
  t.temperature = (int16_t)(bme.readTemperature() * 100.0f);
  t.humidity = (uint16_t)(bme.readHumidity() * 100.0f);
  t.pressure = (uint32_t)(bme.readPressure() * 10.0f);
  t.altitude = (int16_t)(gps.altitude.isValid() ? gps.altitude.meters() * 10.0f : 0);
  t.latitude = (int64_t)(gps.location.isValid() ? gps.location.lat() * 1e8 : 0);
  t.longitude = (int64_t)(gps.location.isValid() ? gps.location.lng() * 1e8 : 0);
  t.battery = 3700; // TODO: read from ADC

  Serial.printf("[TX] pkt=%lu T=%.2f H=%.2f P=%.1f A=%.1f lat=%.6f lon=%.6f bat=%d\n",
                t.packetID,
                t.temperature / 100.0f,
                t.humidity / 100.0f,
                t.pressure / 10.0f,
                t.altitude / 10.0f,
                t.latitude / 1e6f,
                t.longitude / 1e6f,
                t.battery);

  int state = radio.transmit((uint8_t *)&t, sizeof(t));
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.printf("[FAIL] transmit: %d\n", state);
    initGFSKBlocking();
  }
}




void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  SPI.begin(12 /*SCK*/, 13 /*MISO*/, 11 /*MOSI*/, PIN_CS);

  if (!bme.begin())
  {
    Serial.println("[FAIL] BME280 not found, check wiring!");
    while (1)
      delay(10);
  }

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_0_5);

  GPSSerial.begin(9600, SERIAL_8N1, 18 /*RX*/, 17 /*TX*/);

  Serial.println("=========================================");
  Serial.println("   NixiSat Transmitter");
  Serial.printf("   Struct size: %d bytes / %d max\n", sizeof(Telemetry), MAX_FRAME);
  Serial.println("=========================================");

  initGFSKBlocking();
}

void loop()
{
  while (GPSSerial.available())
    gps.encode(GPSSerial.read());

  buildAndSendTelemetry();
  delay(TX_INTERVAL_MS);
}