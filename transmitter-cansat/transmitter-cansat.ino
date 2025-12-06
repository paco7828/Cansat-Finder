#include "LoRaTx.h"
#include "Better-GPS.h"

#define GPS_RX 33

LoRaTx loraTx;
BetterGPS gps;

double lat = 0.000000;
double lon = 0.000000;
double alt = 0.0;
double speed = 0.0;
String formattedTxData = "";

void setup() {
  // Initialization
  Serial.begin(115200);
  loraTx.begin();
  gps.begin(GPS_RX);
  Serial.println("CanSat as transmitter ready!");
}

void loop() {
  gps.update();

  if (gps.hasFix()) {
    lat = gps.getLatitude();
    lon = gps.getLongitude();
    alt = gps.getAltitude();
    speed = gps.getSpeedKmph() / 3.6;  // Convert km/h to m/s to match receiver

    // Format: lat;lon;alt;speed (matches receiver's parseLoRaData expectations)
    formattedTxData = String(lat, 6) + ";" + String(lon, 6) + ";" + String(alt, 1) + ";" + String(speed, 1);

    loraTx.transmit(formattedTxData, 500);
    Serial.println("Sending: " + formattedTxData);
  } else {
    Serial.println("Waiting for GPS fix...");
  }

  delay(200);
}