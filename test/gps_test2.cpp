#include <TinyGPS++.h>
#include <HardwareSerial.h>


HardwareSerial GPS(1);  // use UART1 for the GPS module


#define GPS_RX 16  //defining pins
#define GPS_TX 17 

TinyGPSPlus gps;  

void setup() {
  Serial.begin(115200);  // debugging output
  GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);  // start GPS UART communication at 9600 baud

  Serial.println("GPS Module Initialized. Waiting for GPS signal...");
  int x;
}

void loop() {
  while (GPS.available()) {  // check if GPS data is coming in
    gps.encode(GPS.read());  // process incoming characters
  }

  if (gps.location.isValid()) {  // ensure that we have a valid GPS fix
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);  // print latitude with 6 decimal places
    Serial.print(" Longitude: ");
    Serial.print(gps.location.lng(), 6);  // print longitude with 6 decimal places
    Serial.print(" Altitude: ");
    Serial.print(gps.altitude.meters());  // print altitude in meters
    Serial.print(" Speed: ");
    Serial.print(gps.speed.kmph());  // speed in km/h
    Serial.print(" Course: ");
    Serial.println(gps.course.deg());  // course in degrees
  } 
  else {
    Serial.println("Waiting for a GPS fix...");
  }

  delay(1000); 
}
