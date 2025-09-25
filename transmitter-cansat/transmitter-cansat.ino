#include "LoRaTx.h"
#include "Better-GPS.h"

#define GPS_RX 13

LoRaTx loraTx;
BetterGPS gps;

double lat = 0.000000;
double lon = 0.000000;
String formattedTxCoords = "";

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

    // ✅ Proper string formatting
    formattedTxCoords = String(lat, 6) + ";" + String(lon, 6);

    loraTx.transmit(formattedTxCoords, 500);
    Serial.println("Sending: " + formattedTxCoords);
  } else {
    Serial.println("Waiting for gps fix...");
  }

  delay(200);
}
