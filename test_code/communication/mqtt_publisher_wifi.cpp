#include "PubSubClient.h"
#include "WiFi.h"

// WiFi
const char* ssid = "Nixion-AP";                
const char* wifi_password = "Nixion1234rq-";

// MQTT
const char* mqtt_server = "172.16.0.1"; 
const char* topic = "telemetry/cansat";
const char* mqtt_username = "nixion_user"; // MQTT username
const char* mqtt_password = "Nixion1234rq"; // MQTT password
const char* clientID = "nixisat"; // MQTT client ID

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;

// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient);

// Custom function to connect to the MQTT broker via WiFi
void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection is confirmed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Debugging – Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed…");
  }
}

void setup() {
  Serial.begin(9600);
  connect_MQTT();
}

void loop() {
  if (!client.connected()) {
    connect_MQTT();
  }

  client.loop();

  // Atmospheric & system data
float temperature   = 20.0;      // °C
float humidity      = 80.0;      // %
float pressure      = 1013.25;   // hPa
float co2           = 400.0;     // ppm
float magneticField = 45.7;      // µT
float uvIndex       = 2.5;       // UV index
float irLevel       = 1.2;       // arbitrary units
float battery       = 85.0;      // %
float latitude      = 48.1486;   // degrees North
float longitude     = 17.1077;   // degrees East
float altitude      = 140.0;     // meters
float speed         = 0.0;       // m/s (stationary)


  String payload = "{";
payload += "\"temperature\":" + String(temperature, 2) + ",";
payload += "\"humidity\":" + String(humidity, 2) + ",";
payload += "\"pressure\":" + String(pressure, 2) + ",";
payload += "\"co2\":" + String(co2, 2) + ",";
payload += "\"magnetic_field\":" + String(magneticField, 2) + ",";
payload += "\"uv_index\":" + String(uvIndex, 2) + ",";
payload += "\"ir_level\":" + String(irLevel, 2) + ",";
payload += "\"battery\":" + String(battery, 2) + ",";
payload += "\"latitude\":" + String(latitude, 6) + ",";
payload += "\"longitude\":" + String(longitude, 6) + ",";
payload += "\"altitude\":" + String(altitude, 2) + ",";
payload += "\"speed\":" + String(speed, 2);
payload += "}";



  Serial.println("Publishing data to MQTT:");
  Serial.println(payload);

  // Publish all data to one topic
  if (client.publish(topic, payload.c_str())) {
    Serial.println("Data sent successfully!");
  } else {
    Serial.println("Failed to send data. Reconnecting...");
    connect_MQTT();
  }

  delay(1000);
}