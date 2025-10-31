#include <RadioLib.h>
#include <SPI.h>

#define ss 47
#define rst 41
#define dio0 19
#define mosi 36
#define miso 42
#define sck 40 

int count = 0;
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
    uint32_t pressure;           // hPa * 100
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
  state = radio.setFrequency(869.525);
  state = radio.setBandwidth(125.0);
  state = radio.setSpreadingFactor(7);
  state = radio.setCodingRate(5);
  state = radio.setSyncWord(0xA5);
  state = radio.setOutputPower(17);
  state = radio.setPreambleLength(12);
  state = radio.setCRC(true); 
  
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
  Serial.println("\n\n---Transmitting--");
  Serial.print(F("[SX1276] Transmitting packet ... "));
  
  // Fill telemetry packet
  TelemetryPacket packet;
  packet.packetID = count++;
  packet.temperature       = 2500;         // 25.00°C → *100
  packet.humidity          = 5500;         // 55.00% → *100
  packet.pressure          = 10130;       // 1013.25 hPa → *10
  packet.altitude          = 15000;        // 150.00 m → *100
  packet.speed             = 300;          // 3.00 m/s → *100
  packet.latitude          = 48.6200 * 1e6;
  packet.longitude         = 18.3330 * 1e6;
  packet.latitude_ground   = 48.622576 * 1e6;
  packet.longitude_ground  = 18.336452 * 1e6;
  packet.battery           = 87;           // %

  // Transmit packet
  int state = radio.transmit((uint8_t*)&packet, sizeof(TelemetryPacket));

  uint8_t* ptr = (uint8_t*)&packet;
  size_t size = sizeof(packet);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("\n[SX1276] success!"));
    Serial.print("[SX1276] Sent:\t\t");
    for (size_t i = 0; i < size; ++i) {
      Serial.print("Byte ");
      Serial.print(i);
      Serial.print(": 0x");
      if (ptr[i] < 0x10) Serial.print("0");  // leading zero for single-digit hex
      Serial.println(ptr[i], HEX);
    }
    Serial.println();
    Serial.print(F("[SX1276] Datarate:\t\t"));
    Serial.print(radio.getDataRate());  
    Serial.println(F(" bps"));


    Serial.println("[SX1276] Waiting for ACK...");
    radio.startReceive();
    unsigned long start = millis();
    while (millis() - start < 300) {  
      if (radio.available()) {
        String ack;
        int ackState = radio.readData(ack);
        if (ackState == RADIOLIB_ERR_NONE) {
          Serial.print("[SX1276] Received ACK: ");
          Serial.println(ack);
        } else {
          Serial.print("[SX1276] Failed to read ACK, code ");
          Serial.println(ackState);
        }
        break;
      }
    }
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println(F("too long!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    Serial.println(F("timeout!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }


  radio.startReceive();

  if (receivedFlag) {
    receivedFlag = false;
    String incoming_data;
    int state = radio.readData(incoming_data);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("\n\n---Receiving---");
      Serial.print(F("[SX1276] Data:\t\t"));
      Serial.println(incoming_data);
      Serial.print(F("[SX1276] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));
      Serial.print(F("[SX1276] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));
      Serial.print(F("[SX1276] Frequency error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));
    } else {
      Serial.print(F("[SX1276] Read failed, code "));
      Serial.println(state);
    }

    radio.startReceive();
  }
 

  delay(1000);
}
