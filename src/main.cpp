// Libs & Headers
#include "config.h"
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include "Better-GPS.h"
#include "LoRaRx.h"
#include "Better-Capportal.h"
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "captive-portal-web.h"

// Instances
TFT_eSPI tft = TFT_eSPI();
BetterGPS gps;
LoRaRx *loraRx = nullptr;
LoRaConfig loraConfig;
BetterCapportal capportal;
Preferences preferences;

// SD Card uses HSPI (shared with display)
SPIClass sdSPI(HSPI);

// System state
enum AppState
{
  STATE_TITLE,
  STATE_CONFIG,
  STATE_INITIALIZING,
  STATE_RUNNING
};
AppState appState = STATE_TITLE;

// GPS data
struct GPSData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0; // m/s
  int satellites = 0;
  double hdop = 0.0;
  bool hasFix = false;
  double course = 0.0;
} gpsData;

// CanSat data from LoRa
struct CanSatData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0;
  bool valid = false;
  unsigned long lastReceived = 0;
} canSatData;

// Previous values for selective update
struct PreviousValues
{
  double csLat = -999;
  double csLon = -999;
  double csAlt = -999;
  double csSpeed = -999;
  double gpLat = -999;
  double gpLon = -999;
  double gpAlt = -999;
  double gpSpeed = -999;
  int satellites = -1;
  bool gpsFix = false;
  bool csValid = false;
  double distance = -1;
  double bearing = -999;
  double course = -999;
} prevVals;

// Display variables
unsigned long lastDisplayUpdate = 0;
unsigned long titleStartTime = 0;
const unsigned long TITLE_DURATION = 3000; // 3 seconds
const unsigned long LORA_TIMEOUT = 5000;   // 5 seconds without LoRa data = invalid

// SD Card
bool sdCardAvailable = false;
unsigned long lastSDWrite = 0;
const unsigned long SD_WRITE_INTERVAL = 1000; // Write every 1 second
String currentLogFile = "";

// Initialization variables
unsigned long initStartTime = 0;
int initStep = 0;
const unsigned long INIT_STEP_DURATION = 1000; // 1 second per step

// Flag to track if hardware has been initialized
bool hardwareInitialized = false;

// Config mode variables
int lastClientCount = -1;

// Buzzer variables
unsigned long lastBeepTime = 0;
unsigned long beepInterval = 0;
bool hasPlayedGPSFixBeep = false;
bool startupMelodyPlayed = false;

// Distance thresholds for beeping
const double MAX_DISTANCE = 5000.0;  // 5km
const double MIN_DISTANCE = 30.0;    // 30m
const unsigned long MAX_BEEP_INTERVAL = 10000;  // 10 seconds
const unsigned long MIN_BEEP_INTERVAL = 500;    // 0.5 seconds

// Function prototypes
void saveLoRaConfig();
void loadLoRaConfig();
void handleConfigSubmission(std::map<String, String> &data);
void enterConfigMode();
void initializeHardware();
void updateDisplay();
void drawTitleScreen();
void drawConfigScreen();
void drawRunningScreen();
void drawInitScreen(int step);
void updateValueArea(int x, int y, int w, int h, String value, uint16_t color);
double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void drawArrow(int cx, int cy, int radius, double bearing);
void parseLoRaData(String data);
void initializeSDCard();
void writeToSD();
String buildTelemetryString();
int countSDFiles(const char *dirname);

// Buzzer function prototypes
void playStartupMelody();
void playGPSFixBeep();
void playDistanceBeep();
void beep(int frequency, int duration);
void updateBeepInterval(double distance);

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== CanSat Finder Starting ===");

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  preferences.begin("cansat", false);
  loadLoRaConfig();

  // Initialize display first
  Serial.println("Initializing TFT Display...");
  tft.init();
  tft.setRotation(3); // Landscape mode
  tft.fillScreen(TFT_BLACK);
  Serial.println("Display initialized");

  // CRITICAL: Initialize SD card early, before WiFi
  Serial.println("Initializing SD Card...");
  initializeSDCard();

  titleStartTime = millis();
  drawTitleScreen();
  
  // Play startup melody
  playStartupMelody();
  startupMelodyPlayed = true;
}

void loop()
{
  unsigned long currentTime = millis();

  // State machine
  switch (appState)
  {
  case STATE_TITLE:
    if (currentTime - titleStartTime >= TITLE_DURATION)
    {
      appState = STATE_CONFIG;
      enterConfigMode();
    }
    break;

  case STATE_CONFIG:
  {
    capportal.handle();

    // Update display only if client count changed
    int currentClients = capportal.clientCount();
    if (currentClients != lastClientCount)
    {
      // Only update the client count area
      tft.fillRect(10, 230, 460, 30, TFT_BLACK);
      tft.setTextSize(2);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setCursor(10, 230);
      tft.print("Connected clients: ");
      tft.print(currentClients);
      lastClientCount = currentClients;
    }
    break;
  }

  case STATE_INITIALIZING:
    // Non-blocking initialization sequence
    if (currentTime - initStartTime >= INIT_STEP_DURATION)
    {
      initStep++;
      initStartTime = currentTime;

      if (initStep == 1)
      {
        drawInitScreen(1);
        Serial.println("SD Card status: " + String(sdCardAvailable ? "OK" : "FAIL"));
      }
      else if (initStep == 2)
      {
        drawInitScreen(2);
        Serial.println("Initializing GPS...");
        gps.begin(GPS_RX, GPS_TX, 9600);
        Serial.println("GPS begin() called");
      }
      else if (initStep == 3)
      {
        drawInitScreen(3);
        Serial.println("Initializing LoRa...");
        initializeHardware();
        Serial.println("LoRa initialized");
      }
      else if (initStep == 4)
      {
        drawInitScreen(4);
        Serial.println("Starting system...");
        hardwareInitialized = true;
      }
      else if (initStep >= 5)
      {
        appState = STATE_RUNNING;
        Serial.println("System started!");
        Serial.println("Entering STATE_RUNNING");
        drawRunningScreen();
        hasPlayedGPSFixBeep = false; // Reset GPS fix beep flag
      }
    }
    break;

  case STATE_RUNNING:
    // Update GPS data
    gps.update();

    // Listen for LoRa data
    if (loraRx != nullptr)
    {
      String loraData = loraRx->listen();
      if (loraData.length() > 0)
      {
        // Parse the received data
        parseLoRaData(loraData);
      }
    }

    // Check if LoRa data has timed out
    if (canSatData.valid && (millis() - canSatData.lastReceived > LORA_TIMEOUT))
    {
      Serial.println("LoRa data timeout - marking as invalid");
      canSatData.valid = false;
    }

    // Update GPS data structure
    if (gps.hasFix())
    {
      // Check if this is a new GPS fix
      if (!gpsData.hasFix && !hasPlayedGPSFixBeep)
      {
        playGPSFixBeep();
        hasPlayedGPSFixBeep = true;
      }
      
      gpsData.hasFix = true;
      gpsData.latitude = gps.getLatitude();
      gpsData.longitude = gps.getLongitude();
      gpsData.altitude = gps.getAltitude();
      gpsData.speed = gps.getSpeedKmph() / 3.6; // Convert km/h to m/s
      gpsData.satellites = gps.getSatellites();
      gpsData.hdop = gps.getHdop();
      if (gps.hasCourse())
      {
        gpsData.course = gps.getCourse();
      }
    }
    else
    {
      gpsData.hasFix = false;
      gpsData.latitude = 0.0;
      gpsData.longitude = 0.0;
      gpsData.altitude = 0.0;
      gpsData.speed = 0.0;
      gpsData.satellites = gps.getSatellites();
      gpsData.hdop = gps.getHdop();
      hasPlayedGPSFixBeep = false; // Reset when GPS fix is lost
    }

    // Handle distance-based beeping
    if (gpsData.hasFix && canSatData.valid)
    {
      double distance = calculateDistanceMeters(gpsData.latitude, gpsData.longitude,
                                                canSatData.latitude, canSatData.longitude);
      updateBeepInterval(distance);
      
      if (beepInterval > 0 && (currentTime - lastBeepTime >= beepInterval))
      {
        playDistanceBeep();
        lastBeepTime = currentTime;
      }
    }
    else
    {
      beepInterval = 0; // Stop beeping if no valid data
    }

    // Update display with selective redraw
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
    {
      updateDisplay();
      lastDisplayUpdate = currentTime;
    }

    if (sdCardAvailable && (currentTime - lastSDWrite >= SD_WRITE_INTERVAL))
    {
      writeToSD();
      lastSDWrite = currentTime;
    }
    break;
  }

  delay(10);
}

// ====== BUZZER FUNCTIONS ======

void playStartupMelody()
{
  // CanSat Finder startup melody - triumphant space-themed
  // Notes: C5, E5, G5, C6 (rising arpeggio)
  int melody[] = {523, 659, 784, 1047};
  int durations[] = {150, 150, 150, 300};
  
  for (int i = 0; i < 4; i++)
  {
    beep(melody[i], durations[i]);
    delay(50); // Short pause between notes
  }
}

void playGPSFixBeep()
{
  // Double beep for GPS fix acquired
  beep(1000, 100); // First beep
  delay(100);
  beep(1000, 100); // Second beep
  Serial.println("GPS FIX ACQUIRED - Double beep played");
}

void playDistanceBeep()
{
  // Single short beep for distance indication
  beep(800, 80);
}

void beep(int frequency, int duration)
{
  tone(BUZZER, frequency, duration);
  delay(duration);
  noTone(BUZZER);
}

void updateBeepInterval(double distance)
{
  // Calculate beep interval based on distance
  // Closer = more frequent beeping
  // 5km+ : beep every 10 seconds
  // 30m  : beep every 0.5 seconds
  
  if (distance >= MAX_DISTANCE)
  {
    beepInterval = MAX_BEEP_INTERVAL;
  }
  else if (distance <= MIN_DISTANCE)
  {
    beepInterval = MIN_BEEP_INTERVAL;
  }
  else
  {
    // Linear interpolation between min and max
    // interval = MAX_BEEP_INTERVAL - ((distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE)) * (MAX_BEEP_INTERVAL - MIN_BEEP_INTERVAL)
    double ratio = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
    beepInterval = MAX_BEEP_INTERVAL - (ratio * (MAX_BEEP_INTERVAL - MIN_BEEP_INTERVAL));
  }
  
  // Debug output (uncomment if needed)
  // Serial.printf("Distance: %.1fm, Beep interval: %lums\n", distance, beepInterval);
}

// ====== END BUZZER FUNCTIONS ======

// Parse LoRa data string (format: "lat;lon" or "lat;lon;alt;speed")
void parseLoRaData(String data)
{
  data.trim();

  if (data.length() == 0)
  {
    return;
  }

  Serial.println("Parsing LoRa data: " + data);

  // First, check if it's a valid coordinate string
  int semicolonCount = 0;
  for (int i = 0; i < data.length(); i++)
  {
    if (data.charAt(i) == ';')
    {
      semicolonCount++;
    }
  }

  // We need at least one semicolon for lat;lon format
  if (semicolonCount == 0)
  {
    Serial.println("Invalid format - no semicolons");
    return;
  }

  // Parse based on number of semicolons
  int firstSemi = data.indexOf(';');
  String latStr = data.substring(0, firstSemi);

  // Check if first part is a valid number
  double lat = latStr.toDouble();
  if (lat == 0.0 && latStr != "0" && latStr != "0.0")
  {
    // Invalid latitude
    Serial.println("Invalid latitude: " + latStr);
    return;
  }

  if (semicolonCount == 1)
  {
    // Format: lat;lon
    String lonStr = data.substring(firstSemi + 1);
    double lon = lonStr.toDouble();

    canSatData.latitude = lat;
    canSatData.longitude = lon;
    canSatData.altitude = 0.0;
    canSatData.speed = 0.0;

    Serial.printf("Parsed 2-part: Lat=%.6f, Lon=%.6f\n", lat, lon);
  }
  else if (semicolonCount >= 2)
  {
    // Format: lat;lon;alt or lat;lon;alt;speed
    int secondSemi = data.indexOf(';', firstSemi + 1);
    String lonStr = data.substring(firstSemi + 1, secondSemi);
    double lon = lonStr.toDouble();

    canSatData.latitude = lat;
    canSatData.longitude = lon;

    if (semicolonCount >= 3)
    {
      int thirdSemi = data.indexOf(';', secondSemi + 1);
      if (thirdSemi != -1)
      {
        // Format: lat;lon;alt;speed
        String altStr = data.substring(secondSemi + 1, thirdSemi);
        String speedStr = data.substring(thirdSemi + 1);

        canSatData.altitude = altStr.toDouble();
        canSatData.speed = speedStr.toDouble();

        Serial.printf("Parsed 4-part: Lat=%.6f, Lon=%.6f, Alt=%.1f, Speed=%.1f\n",
                      lat, lon, canSatData.altitude, canSatData.speed);
      }
      else
      {
        // Format: lat;lon;alt
        String altStr = data.substring(secondSemi + 1);
        canSatData.altitude = altStr.toDouble();
        canSatData.speed = 0.0;

        Serial.printf("Parsed 3-part: Lat=%.6f, Lon=%.6f, Alt=%.1f\n",
                      lat, lon, canSatData.altitude);
      }
    }
    else
    {
      // Only lat;lon (with trailing semicolon?)
      canSatData.altitude = 0.0;
      canSatData.speed = 0.0;
    }
  }

  canSatData.valid = true;
  canSatData.lastReceived = millis();
}

// Draw title screen
void drawTitleScreen()
{
  tft.fillScreen(TFT_BLACK);

  // Draw title - larger and centered
  tft.setTextSize(5);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);

  // Center "CanSat" (6 chars * 30px per char = 180px width)
  int cansatWidth = 6 * 30;
  tft.setCursor((480 - cansatWidth) / 2, 60);
  tft.println("CanSat");

  // Center "Finder" (6 chars * 30px per char = 180px width)
  int finderWidth = 6 * 30;
  tft.setCursor((480 - finderWidth) / 2, 120);
  tft.println("Finder");

  // Draw S.E.A.T.
  tft.setTextSize(3);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  // Center "S.E.A.T." (8 chars * 18px per char = 144px width)
  int seatWidth = 8 * 18;
  tft.setCursor((480 - seatWidth) / 2, 190);
  tft.println("S.E.A.T.");

  // Loading text
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Center "Starting..." (12 chars * 12px per char = 144px width)
  int startWidth = 12 * 12;
  tft.setCursor((480 - startWidth) / 2, 250);
  tft.println("Starting...");
}

// Draw config screen
void drawConfigScreen()
{
  tft.fillScreen(TFT_BLACK);

  // Title
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(120, 10);
  tft.println("CONFIG MODE");

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // WiFi info - moved down with gap from title
  tft.setCursor(10, 60);
  tft.println("WiFi AP: " + String(SSID));
  tft.setCursor(10, 85);
  tft.println("IP: 4.3.2.1");

  // Current settings - no extra gap
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 115);
  tft.println("Current LoRa Settings:");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 140);
  tft.print("Freq: ");
  tft.print(loraConfig.frequency);
  tft.println(" Hz");

  tft.setCursor(10, 165);
  tft.print("BW: ");
  tft.print(loraConfig.bandwidth);
  tft.print(" kHz  Sync: ");
  tft.println(loraConfig.sync);

  tft.setCursor(10, 190);
  tft.print("Baudrate: ");
  tft.print(loraConfig.baudrate);
  tft.println(" bps");

  // Client count (will be updated separately)
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 230);
  tft.print("Connected clients: ");
  tft.print(capportal.clientCount());
  lastClientCount = capportal.clientCount();

  // Instructions
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 275);
  tft.println("Configure at: http://4.3.2.1");
}

// Draw initialization screen
void drawInitScreen(int step)
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(130, 60);
  tft.println("Initializing...");

  tft.setTextSize(2);
  switch (step)
  {
  case 1:
    tft.setCursor(100, 130);
    tft.println("SD Card...");
    break;
  case 2:
    tft.setCursor(100, 120);
    tft.setTextColor(sdCardAvailable ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print("SD [");
    tft.print(sdCardAvailable ? "OK" : "FAIL");
    tft.println("]");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, 150);
    tft.println("GPS module...");
    break;
  case 3:
    tft.setCursor(100, 110);
    tft.setTextColor(sdCardAvailable ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print("SD [");
    tft.print(sdCardAvailable ? "OK" : "FAIL");
    tft.println("]");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, 140);
    tft.println("GPS [OK]");
    tft.setCursor(100, 170);
    tft.println("LoRa...");
    break;
  case 4:
    tft.setCursor(100, 100);
    tft.setTextColor(sdCardAvailable ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print("SD [");
    tft.print(sdCardAvailable ? "OK" : "FAIL");
    tft.println("]");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, 130);
    tft.println("GPS [OK]");
    tft.setCursor(100, 160);
    tft.println("LoRa [OK]");
    tft.setCursor(100, 190);
    tft.println("Starting...");
    break;
  }
}

// Draw running screen - initial setup only
void drawRunningScreen()
{
  tft.fillScreen(TFT_BLACK);

  // Header - centered in one line
  tft.setTextSize(3);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  // "CanSat Finder" = 13 chars * 18px = 234px width
  int headerWidth = 13 * 18;
  tft.setCursor((480 - headerWidth) / 2, 5);
  tft.println("CanSat Finder");

  tft.drawLine(0, 35, 480, 35, TFT_WHITE);

  // Static labels
  int yPos = 45;
  tft.setTextSize(2);

  // CanSat section
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(5, yPos);
  tft.println("CanSat Data:");

  yPos += 25;
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, yPos);
  tft.println("Lat:");
  tft.setCursor(10, yPos + 20);
  tft.println("Lon:");
  tft.setCursor(10, yPos + 40);
  tft.println("Alt:");
  tft.setCursor(10, yPos + 60);
  tft.println("Speed:");

  yPos += 90;

  // Local Data section
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(5, yPos);
  tft.println("Local Data:");

  yPos += 25;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, yPos);
  tft.println("Lat:");
  tft.setCursor(10, yPos + 20);
  tft.println("Lon:");
  tft.setCursor(10, yPos + 40);
  tft.println("Alt:");
  tft.setCursor(10, yPos + 60);
  tft.println("Speed:");
  tft.setCursor(10, yPos + 80);
  tft.println("Sat:");

  // Draw arrow circle in center-right area (moved up)
  tft.drawCircle(380, 160, 70, TFT_WHITE);

  // Distance label below arrow
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(320, 245);
  tft.println("Distance:");

  tft.setTextSize(1);
  if (sdCardAvailable)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, 305);
    tft.println("SD: OK");
  }
  else
  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(10, 305);
    tft.println("SD: FAIL");
  }

  // Reset previous values to force initial update
  prevVals.csLat = -999;
}

// Update display - selective updates only
void updateDisplay()
{
  int yPos = 70;
  uint16_t csColor = canSatData.valid ? TFT_GREEN : TFT_RED;

  // Update CanSat values if changed
  if (prevVals.csLat != canSatData.latitude)
  {
    updateValueArea(90, yPos, 160, 18, String(canSatData.latitude, 6), csColor);
    prevVals.csLat = canSatData.latitude;
  }
  if (prevVals.csLon != canSatData.longitude)
  {
    updateValueArea(90, yPos + 20, 160, 18, String(canSatData.longitude, 6), csColor);
    prevVals.csLon = canSatData.longitude;
  }
  if (prevVals.csAlt != canSatData.altitude)
  {
    updateValueArea(90, yPos + 40, 160, 18, String(canSatData.altitude, 1) + " m", csColor);
    prevVals.csAlt = canSatData.altitude;
  }
  if (prevVals.csSpeed != canSatData.speed)
  {
    updateValueArea(90, yPos + 60, 160, 18, String(canSatData.speed, 1) + " m/s", csColor);
    prevVals.csSpeed = canSatData.speed;
  }

  // Update validity status if changed
  if (prevVals.csValid != canSatData.valid)
  {
    prevVals.csValid = canSatData.valid;
  }

  yPos = 185;
  uint16_t gpsColor = gpsData.hasFix ? TFT_GREEN : TFT_RED;

  // Update GPS values if changed
  if (prevVals.gpLat != gpsData.latitude)
  {
    updateValueArea(90, yPos, 160, 18, String(gpsData.latitude, 6), gpsColor);
    prevVals.gpLat = gpsData.latitude;
  }
  if (prevVals.gpLon != gpsData.longitude)
  {
    updateValueArea(90, yPos + 20, 160, 18, String(gpsData.longitude, 6), gpsColor);
    prevVals.gpLon = gpsData.longitude;
  }
  if (prevVals.gpAlt != gpsData.altitude)
  {
    updateValueArea(90, yPos + 40, 160, 18, String(gpsData.altitude, 1) + " m", gpsColor);
    prevVals.gpAlt = gpsData.altitude;
  }
  if (prevVals.gpSpeed != gpsData.speed)
  {
    updateValueArea(90, yPos + 60, 160, 18, String(gpsData.speed, 1) + " m/s", gpsColor);
    prevVals.gpSpeed = gpsData.speed;
  }

  // Satellites
  if (prevVals.satellites != gpsData.satellites)
  {
    updateValueArea(90, yPos + 80, 80, 18, String(gpsData.satellites), TFT_WHITE);
    prevVals.satellites = gpsData.satellites;
  }

  // Distance and arrow
  if (gpsData.hasFix && canSatData.valid)
  {
    double distance = calculateDistanceMeters(gpsData.latitude, gpsData.longitude,
                                              canSatData.latitude, canSatData.longitude);
    double absoluteBearing = calculateBearing(gpsData.latitude, gpsData.longitude,
                                              canSatData.latitude, canSatData.longitude);

    // Calculate RELATIVE bearing (target direction relative to your heading)
    double relativeBearing = absoluteBearing - gpsData.course;

    // Normalize to 0-360 range
    while (relativeBearing < 0)
      relativeBearing += 360;
    while (relativeBearing >= 360)
      relativeBearing -= 360;

    if (abs(prevVals.distance - distance) > 0.5)
    {
      String distStr;
      if (distance < 1000)
      {
        distStr = String((int)distance) + " m";
      }
      else
      {
        distStr = String(distance / 1000.0, 2) + " km";
      }
      updateValueArea(300, 265, 150, 18, distStr, TFT_YELLOW);
      prevVals.distance = distance;
    }

    // Update arrow if bearing OR course changed significantly
    bool needsRedraw = false;
    double bearingToUse = absoluteBearing;

    // If moving with valid course, use relative bearing
    if (gpsData.speed > 1.0 && gps.hasCourse())
    {
      bearingToUse = relativeBearing;
      needsRedraw = (abs(prevVals.bearing - absoluteBearing) > 1.0 ||
                     abs(prevVals.course - gpsData.course) > 2.0);
    }
    else
    {
      // Stationary or no course - use absolute bearing (compass mode)
      bearingToUse = absoluteBearing;
      needsRedraw = (abs(prevVals.bearing - absoluteBearing) > 1.0);
    }

    if (needsRedraw)
    {
      // Clear arrow area
      tft.fillCircle(380, 160, 69, TFT_BLACK);
      tft.drawCircle(380, 160, 70, TFT_WHITE);

      // Draw arrow
      drawArrow(380, 160, 60, bearingToUse);

      prevVals.bearing = absoluteBearing;
      prevVals.course = gpsData.course;
    }
  }
  else
  {
    if (prevVals.distance != -1)
    {
      updateValueArea(300, 265, 150, 18, "---", TFT_YELLOW);
      prevVals.distance = -1;
      // Clear arrow
      tft.fillCircle(380, 160, 69, TFT_BLACK);
      tft.drawCircle(380, 160, 70, TFT_WHITE);
      prevVals.bearing = -999;
      prevVals.course = -999;
    }
  }
}

// Helper to update value area
void updateValueArea(int x, int y, int w, int h, String value, uint16_t color)
{
  tft.fillRect(x, y, w, h, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  tft.println(value);
}

// Calculate bearing from point 1 to point 2
double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) -
             sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double bearing = atan2(y, x);
  bearing = degrees(bearing);
  bearing = fmod((bearing + 360.0), 360.0);
  return bearing;
}

double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2)
{
  // Earth's radius in meters
  const double R = 6371000.0;

  // Convert degrees to radians
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double deltaPhi = radians(lat2 - lat1);
  double deltaLambda = radians(lon2 - lon1);

  // Haversine formula
  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
             cos(phi1) * cos(phi2) *
                 sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;
}

// Draw arrow pointing to bearing
void drawArrow(int cx, int cy, int radius, double bearing)
{
  // Convert bearing to radians
  double angle = radians(bearing - 90); // -90 to make 0° point up

  // Arrow tip
  int tipX = cx + radius * cos(angle);
  int tipY = cy + radius * sin(angle);

  // Arrow base (opposite side)
  int baseX = cx - radius * 0.3 * cos(angle);
  int baseY = cy - radius * 0.3 * sin(angle);

  // Arrow wings
  double wingAngle1 = angle + radians(150);
  double wingAngle2 = angle - radians(150);

  int wing1X = cx + radius * 0.6 * cos(wingAngle1);
  int wing1Y = cy + radius * 0.6 * sin(wingAngle1);

  int wing2X = cx + radius * 0.6 * cos(wingAngle2);
  int wing2Y = cy + radius * 0.6 * sin(wingAngle2);

  // Draw arrow
  tft.fillTriangle(tipX, tipY, wing1X, wing1Y, baseX, baseY, TFT_GREEN);
  tft.fillTriangle(tipX, tipY, wing2X, wing2Y, baseX, baseY, TFT_GREEN);

  // Draw center dot
  tft.fillCircle(cx, cy, 4, TFT_RED);
}

// Handle configuration submission
void handleConfigSubmission(std::map<String, String> &data)
{
  Serial.println("LoRa configuration received:");

  String newFreq = data["frequency"];
  String newBW = data["bandwidth"];
  String newSync = data["sync"];
  String newBaudStr = data["baudrate"];

  if (newFreq.length() > 0)
    loraConfig.frequency = newFreq;
  if (newBW.length() > 0)
    loraConfig.bandwidth = newBW;
  if (newSync.length() > 0)
    loraConfig.sync = newSync;

  long newBaud = newBaudStr.toInt();
  if (newBaud > 0)
    loraConfig.baudrate = newBaud;

  Serial.printf("New config - Freq: %s, BW: %s, Sync: %s, Baud: %ld\n",
                loraConfig.frequency.c_str(), loraConfig.bandwidth.c_str(),
                loraConfig.sync.c_str(), loraConfig.baudrate);

  saveLoRaConfig();

  capportal.stop();
  delay(100);

  appState = STATE_INITIALIZING;
  initStartTime = millis();
  initStep = 0;

  Serial.println("Starting initialization sequence...");
}

// Enter configuration mode
void enterConfigMode()
{
  Serial.println("Entering configuration mode...");
  String configHTML = generateConfigHTML(loraConfig);
  capportal.setHTML(configHTML);
  capportal.onSubmit(handleConfigSubmission);
  capportal.begin(SSID, PASSWRD, AP_TIMEOUT);

  drawConfigScreen();
  Serial.println("Config mode active");
}

// Initialize hardware (LoRa)
void initializeHardware()
{
  if (loraRx != nullptr)
  {
    delete loraRx;
    loraRx = nullptr;
  }

  // ESP32-S3 pins: RX=18, TX=17 for receiver
  loraRx = new LoRaRx(18, 17, loraConfig.baudrate);

  // Use the configured parameters
  loraRx->begin(loraConfig.frequency, loraConfig.bandwidth, loraConfig.sync);

  Serial.println("LoRa hardware initialized with:");
  Serial.println("  Frequency: " + loraConfig.frequency + " Hz");
  Serial.println("  Bandwidth: " + loraConfig.bandwidth + " kHz");
  Serial.println("  Sync Word: " + loraConfig.sync);
  Serial.println("  Baudrate: " + String(loraConfig.baudrate));
}

// Load LoRa configuration
void loadLoRaConfig()
{
  loraConfig.frequency = preferences.getString("frequency", "865375000");
  loraConfig.bandwidth = preferences.getString("bandwidth", "250");
  loraConfig.sync = preferences.getString("sync", "12");
  loraConfig.baudrate = preferences.getLong("baudrate", 115200);
  Serial.printf("Loaded config - Freq: %s, BW: %s, Sync: %s, Baud: %ld\n",
                loraConfig.frequency.c_str(), loraConfig.bandwidth.c_str(),
                loraConfig.sync.c_str(), loraConfig.baudrate);
}

// Save LoRa configuration
void saveLoRaConfig()
{
  preferences.putString("frequency", loraConfig.frequency);
  preferences.putString("bandwidth", loraConfig.bandwidth);
  preferences.putString("sync", loraConfig.sync);
  preferences.putLong("baudrate", loraConfig.baudrate);
  Serial.println("Configuration saved to preferences!");
}

void initializeSDCard()
{
  if (!SD_ENABLED)
  {
    sdCardAvailable = false;
    Serial.println("SD Card disabled in config");
    return;
  }

  Serial.println("Initializing SD Card on HSPI...");
  Serial.printf("  CS: %d, SCK: %d, MISO: %d, MOSI: %d\n",
                SD_CS_PIN, SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN);

  // Initialize HSPI for SD card with custom pins
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  // Give SPI bus time to stabilize
  delay(100);

  // Try to initialize SD card
  if (!SD.begin(SD_CS_PIN, sdSPI))
  {
    Serial.println("SD Card mount failed!");
    sdCardAvailable = false;
    return;
  }

  // Verify SD card is accessible
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    sdCardAvailable = false;
    SD.end();
    return;
  }

  // Print card info
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  sdCardAvailable = true;
  Serial.println("SD Card initialized successfully!");

  // Create logs directory if it doesn't exist
  if (!SD.exists("/logs"))
  {
    SD.mkdir("/logs");
    Serial.println("Created /logs directory");
  }

  // Create new log file with timestamp
  int fileCount = countSDFiles("/logs");
  currentLogFile = "/logs/telemetry_" + String(fileCount + 1) + ".csv";

  File file = SD.open(currentLogFile, FILE_WRITE);
  if (file)
  {
    String header = "Time,CS_Lat,CS_Lon,CS_Alt,CS_Speed,CS_Valid,GP_Lat,GP_Lon,GP_Alt,GP_Speed,GP_Sats,GP_Fix,Distance,Bearing";
    file.println(header);
    file.close();
    Serial.println("Created telemetry file: " + currentLogFile);
  }
  else
  {
    Serial.println("Failed to create telemetry file!");
    sdCardAvailable = false;
  }
}

// Helper function to count files in directory
int countSDFiles(const char *dirname)
{
  File dir = SD.open(dirname);
  if (!dir || !dir.isDirectory())
  {
    return 0;
  }

  int count = 0;
  File file = dir.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
    {
      count++;
    }
    file.close();
    file = dir.openNextFile();
  }
  dir.close();
  return count;
}

String buildTelemetryString()
{
  String data = String(millis()) + ",";
  data += String(canSatData.latitude, 6) + ",";
  data += String(canSatData.longitude, 6) + ",";
  data += String(canSatData.altitude, 2) + ",";
  data += String(canSatData.speed, 2) + ",";
  data += (canSatData.valid ? "1" : "0") + String(",");
  data += String(gpsData.latitude, 6) + ",";
  data += String(gpsData.longitude, 6) + ",";
  data += String(gpsData.altitude, 2) + ",";
  data += String(gpsData.speed, 2) + ",";
  data += String(gpsData.satellites) + ",";
  data += (gpsData.hasFix ? "1" : "0") + String(",");

  if (gpsData.hasFix && canSatData.valid)
  {
    double dist = calculateDistanceMeters(gpsData.latitude, gpsData.longitude,
                                          canSatData.latitude, canSatData.longitude);
    double bear = calculateBearing(gpsData.latitude, gpsData.longitude,
                                   canSatData.latitude, canSatData.longitude);
    data += String(dist, 2) + "," + String(bear, 2);
  }
  else
  {
    data += "0.00,0.00";
  }

  return data;
}

void writeToSD()
{
  if (!sdCardAvailable || currentLogFile.length() == 0)
    return;

  String data = buildTelemetryString();

  // Append to current log file
  File file = SD.open(currentLogFile, FILE_APPEND);
  if (file)
  {
    file.println(data);
    file.close();
    // Uncomment for debugging:
    // Serial.println("SD: " + data);
  }
  else
  {
    Serial.println("Failed to write to SD card: " + currentLogFile);
    // Try to reinitialize on next write
    sdCardAvailable = false;
  }
}