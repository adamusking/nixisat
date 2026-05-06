#include <Arduino.h>
#include <WiFi.h>

// --- Configuration ---
const char* ssid = "Nixion";
const char* password = "";
const char* serverIP = "10.0.0.2"; // Replace with your Laptop's IP
const uint16_t serverPort = 6967;
const int testDuration = 15000; // 15 seconds
const size_t bufferSize = 4096;

uint8_t dataBuffer[bufferSize];
WiFiClient client;

void setup() {
    Serial.begin(115200);
    
    // Fill buffer with random "data"
    for(int i=0; i<bufferSize; i++) dataBuffer[i] = (uint8_t)random(0, 255);

    WiFi.begin(ssid);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
}

void loop() {
    if (client.connect(serverIP, serverPort)) {
        Serial.println("Connected to Ground Station.");

        // --- PHASE 1: Send Data to GS (Uplink) ---
        Serial.println("Starting Uplink...");
        unsigned long startTest = millis();
        while (millis() - startTest < testDuration) {
            client.write(dataBuffer, bufferSize);
        }
        Serial.println("Uplink complete.");

        // --- PHASE 2: Receive Data from GS (Downlink) ---
        Serial.println("Starting Downlink measurement...");
        unsigned long totalBytesReceived = 0;
        startTest = millis();
        
        while (millis() - startTest < testDuration) {
            while (client.available() > 0) {
                int len = client.read(dataBuffer, bufferSize);
                if (len > 0) totalBytesReceived += len;
            }
        }
        
        float elapsedSec = (millis() - startTest) / 1000.0;
        float speedMbps = (totalBytesReceived * 8.0) / (elapsedSec * 1000000.0);

        // --- PHASE 3: Send Result back to GS ---
        String result = "Downlink Speed: " + String(speedMbps) + " Mbps";
        client.println(result);
        Serial.println(result);

        client.stop();
        Serial.println("Test finished. Sleeping for 30s.");
        delay(30000); 
    } else {
        Serial.println("Connection failed. Retrying in 5s...");
        delay(5000);
    }
}