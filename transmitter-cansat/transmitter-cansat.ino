// ============================================================================
//  CanSat_GPS_Minimal.ino – Onallo, minimalista CanSat firmware (csak GPS)
// ============================================================================
//  Cel: kizarolag GPS adatot dolgoz fel es kuld LoRa-n a HT-CT62-n keresztul,
//  a CS26 "02;" (Avionics) csomagformatum szerint. A Finder oldali
//  parseAvionics() (CS26_TelemetryParser.h) valtoztatas nelkul fogadja.
//
//  HW: Seeed XIAO ESP32S3 Plus
//    GPS  <-> ESP32:  IO39 = GPS TX  -> ESP32 RX (UART2)
//                      IO40 = GPS RX <- ESP32 TX (UART2)
//    HT-CT62 <-> ESP32: IO41 = HTCT62 TX -> ESP32 RX (UART1 / Serial1)
//                        IO42 = HTCT62 RX <- ESP32 TX (UART1 / Serial1)
//
//  A GPS es a LoRa KULON hardver-UART periferiat hasznal (UART2 vs UART1),
//  igy nincs periferia-utkozes -- ugyanaz a mintat kovetjuk, mint a Finderen.
//
//  CSV formatum (CS26_TelemetryBuilder.h::buildAvionics() alapjan, 14 mezo):
//    02;seq;ts;lat;lon;alt_gps;alt_baro;speed;course;sats;gps_age_ms;hour;min;sec
//
//  Ez a firmware nem rendelkezik barometrikus szenzorral -> alt_baro mindig "?"
//  (NaN), amit a Finder parseAvionics()-a helyesen NAN-kent olvas be es a
//  main.cpp jelenleg ugyis a bme680_alt_m mezot hasznalja megjelenitesre --
//  ha ez itt mindig NaN, a Finder kijelzon az Altitude mezo "nan m"-et fog
//  mutatni, amig at nem allitod gps_alt_m-re (lasd megjegyzes lejjebb).
// ============================================================================

#include <Arduino.h>
#include <TinyGPSPlus.h>

// ---------------------------------------------------------------------------
// Pin- es UART-kiosztas
// ---------------------------------------------------------------------------
constexpr uint8_t GPS_RX_PIN = 39;  // ESP32 RX <- GPS TX
constexpr uint8_t GPS_TX_PIN = 40;  // ESP32 TX -> GPS RX
constexpr uint32_t GPS_BAUD = 9600;

constexpr uint8_t LORA_RX_PIN = 41;  // ESP32 RX <- HT-CT62 TXD
constexpr uint8_t LORA_TX_PIN = 42;  // ESP32 TX -> HT-CT62 RXD
constexpr uint32_t LORA_BAUD = 115200;

// GPS sajat UART2-n, LoRa (HT-CT62) a Serial1-en (UART1) -- kulon periferia,
// nincs utkozes (ugyanaz a minta, mint a Finder main.cpp-jeben).
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// ---------------------------------------------------------------------------
// Kuldesi utemezes
// ---------------------------------------------------------------------------
constexpr uint32_t SEND_INTERVAL_MS = 500;  // 2 Hz avionics kuldes
uint32_t lastSend_ms = 0;
uint16_t seq = 0;

// ---------------------------------------------------------------------------
// NaN-biztos float -> string helper (a CS26_TelemetryBuilder.h _fmt()-jenek
// megfeleloje, ide masolva, hogy ez a firmware onallo maradjon a teljes
// SEAT_CS26_Telemetry lib linkelese nelkul).
// ---------------------------------------------------------------------------
int fmtFloat(char *buf, int len, float v, int dec = 1) {
  if (isnan(v))
    return snprintf(buf, len, "?");
  char fmt[10];
  snprintf(fmt, sizeof(fmt), "%%.%df", dec);
  return snprintf(buf, len, fmt, v);
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("CanSat GPS Minimal - starting...");

  // GPS UART2-n
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("GPS UART2 initialized: RX=%d TX=%d @ %lu baud\n",
                GPS_RX_PIN, GPS_TX_PIN, (unsigned long)GPS_BAUD);

  // HT-CT62 (LoRa shell) Serial1-en (UART1)
  Serial1.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  Serial.printf("LoRa UART1 initialized: RX=%d TX=%d @ %lu baud\n",
                LORA_RX_PIN, LORA_TX_PIN, (unsigned long)LORA_BAUD);

  // HT-CT62 explicit konfiguralasa induláskor -- ugyanaz a beallitas, mint
  // amit a Finder oldalon hasznalunk (868.0MHz/125kHz/SF7/CR5/sync0x12/14dBm).
  delay(300);  // HT-CT62 sajat boot-idejenek kivarasa
  String cfgCmd = "set_config 868.0,125,7,5,12,14,8,1.6,2";
  Serial1.println(cfgCmd);
  Serial.println("LORA_TX: " + cfgCmd);
  delay(150);  // radio-reinit ideje a HT-CT62-n
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // GPS NMEA folyamatos dekodolasa
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

  // HT-CT62 valaszainak echo-zasa a USB Serialra (diagnosztikahoz)
  while (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
      Serial.println("LORA_RX: " + line);
  }

  // Serial -> HT-CT62 forwarding (diagnosztikahoz, pl. get_freq/get_config)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0)
      Serial1.println(cmd);
  }

  // 1 Hz-es avionics kuldes
  uint32_t now = millis();
  if (now - lastSend_ms >= SEND_INTERVAL_MS) {
    lastSend_ms = now;
    sendAvionics();
  }
}

// ---------------------------------------------------------------------------
// sendAvionics() -- CS26 "02;" csomag osszeallitasa es kuldese
//
// Mezosorrend (buildAvionics()-szal egyezoen, 14 mezo):
//   02;seq;ts;lat;lon;alt_gps;alt_baro;speed;course;sats;gps_age_ms;hour;min;sec
// ---------------------------------------------------------------------------
void sendAvionics() {
  bool hasFix = gps.location.isValid();

  char lat[12], lon[12], altGps[10], altBaro[10], spd[8], crs[8], age[8];

  fmtFloat(lat, sizeof(lat), hasFix ? (float)gps.location.lat() : NAN, 5);
  fmtFloat(lon, sizeof(lon), hasFix ? (float)gps.location.lng() : NAN, 5);
  fmtFloat(altGps, sizeof(altGps), hasFix ? (float)gps.altitude.meters() : NAN, 0);
  fmtFloat(altBaro, sizeof(altBaro), NAN, 0);  // nincs barometrikus szenzor -> mindig "?"
  fmtFloat(spd, sizeof(spd), hasFix ? (float)gps.speed.kmph() : NAN, 1);
  fmtFloat(crs, sizeof(crs), (hasFix && gps.course.isValid()) ? (float)gps.course.deg() : NAN, 0);
  fmtFloat(age, sizeof(age), hasFix ? (float)gps.location.age() : NAN, 0);

  uint8_t sats = (uint8_t)gps.satellites.value();
  uint8_t hour = gps.time.isValid() ? gps.time.hour() : 0;
  uint8_t minu = gps.time.isValid() ? gps.time.minute() : 0;
  uint8_t sec = gps.time.isValid() ? gps.time.second() : 0;

  char buf[160];
  snprintf(buf, sizeof(buf), "02;%u;%lu;%s;%s;%s;%s;%s;%s;%u;%s;%02u;%02u;%02u",
           ++seq, (unsigned long)millis(),
           lat, lon, altGps, altBaro, spd, crs, sats, age,
           hour, minu, sec);

  String payload(buf);
  String cmd = "send \"" + payload + "\"";
  Serial1.println(cmd);
  Serial.println("LORA_TX: " + cmd);

  // Diagnosztikai osszefoglalo: van-e fix, hany mullhollyv latszik
  Serial.printf("GPS: fix=%s sats=%u lat=%s lon=%s\n",
                hasFix ? "YES" : "NO", sats, lat, lon);
}
