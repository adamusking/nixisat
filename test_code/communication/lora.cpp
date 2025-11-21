#include <RadioLib.h>
#include <SPI.h>

#define ss 47
#define rst 41
#define dio0 19
#define mosi 36
#define miso 42
#define sck 40 

uint16_t count = 0;  // packetID counter
SX1276 radio = new Module(ss, dio0, rst);

// Flag for receiving (unused but kept for reference)
volatile bool receivedFlag = false;

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

// Telemetry packet structure
typedef struct {
    uint16_t packetID;
    uint16_t temperature;        // °C * 100
    uint16_t humidity;           // % * 100
    uint16_t pressure;           // hPa * 100
    uint16_t altitude;           // meters * 100
    uint16_t speed;              // m/s * 100
    uint32_t latitude;           // degrees * 1e6
    uint32_t longitude;          // degrees * 1e6
    uint32_t latitude_ground;    // degrees * 1e6
    uint32_t longitude_ground;   // degrees * 1e6
    uint8_t  battery;            // % battery level
} __attribute__((packed)) TelemetryPacket;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Transceiver");

  SPI.begin(sck, miso, mosi, ss);

  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // Configure LoRa parameters
  radio.setFrequency(869.525);
  radio.setBandwidth(125.0);
  radio.setSpreadingFactor(7);
  radio.setCodingRate(5);
  radio.setSyncWord(0xA5);
  radio.setOutputPower(17);
  radio.setPreambleLength(12);
  radio.setCRC(true);

  // Set callback for receiving (unused)
  radio.setPacketReceivedAction(setFlag);

  Serial.print(F("[SX1276] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
}

void loop() {
  Serial.println("\n\n---Transmitting---");

  // Fill telemetry packet
  TelemetryPacket packet;
  packet.packetID        = count;       // only increment after successful transmit
  packet.temperature     = 2500;        
  packet.humidity        = 5500;        
  packet.pressure        = 9060;        
  packet.altitude        = 15000;       
  packet.speed           = 300;         
  packet.latitude        = (uint32_t)(48.6200 * 1e6);
  packet.longitude       = (uint32_t)(18.3330 * 1e6);
  packet.latitude_ground = (uint32_t)(48.622576 * 1e6);
  packet.longitude_ground= (uint32_t)(18.336452 * 1e6);
  packet.battery         = 87;          

  // Transmit packet
  int state = radio.transmit((uint8_t*)&packet, sizeof(TelemetryPacket));

  uint8_t* ptr = (uint8_t*)&packet;
  size_t size = sizeof(packet);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1276] Transmission SUCCESS!"));
    Serial.print("[SX1276] Sent packetID: ");
    Serial.println(count);
    count++;  // increment only after successful transmission

    Serial.print("[SX1276] Bytes sent: ");
    for (size_t i = 0; i < size; ++i) {
      Serial.print("0x");
      if (ptr[i] < 0x10) Serial.print("0");
      Serial.print(ptr[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print(F("[SX1276] Data rate: "));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println(F("[SX1276] Packet too long!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    Serial.println(F("[SX1276] Transmission timeout!"));
  } else {
    Serial.print(F("[SX1276] Transmission failed, code "));
    Serial.println(state);
  }

  // Delay 1 second before next packet
  delay(1000);
}
