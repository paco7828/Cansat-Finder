# CanSat Finder

An ESP32-based ground station tracker for the S.E.A.T. CanSat team. Receives telemetry from a CanSat via LoRa, displays live GPS and telemetry data on a 480x320 TFT, calculates bearing and distance, and logs everything to an SD card. Configurable over a captive portal WiFi AP.

---

## Hardware

| Component | Description |
|-----------|-------------|
| MCU | ESP32 |
| Display | TFT_eSPI-compatible 480x320 display (landscape) |
| GPS | UART GPS module (NEO-6M or compatible) |
| LoRa | UART-based LoRa module (RN2483 command set) |
| Storage | MicroSD card (HSPI) |
| Audio | Passive buzzer |

### Pin Assignment

| Signal | GPIO |
|--------|------|
| GPS RX (ESP32) | 5 |
| GPS TX (ESP32) | 6 |
| LoRa RX (ESP32) | 18 |
| LoRa TX (ESP32) | 17 |
| Buzzer | 7 |
| SD CS | 3 |
| SD SCK | 14 |
| SD MISO | 15 |
| SD MOSI | 13 |

---

## Features

### LoRa Reception

- Receives semicolon-delimited telemetry strings: `latitude;longitude;altitude;speed`
- Auto-detects data timeout (5 seconds) and marks CanSat data as invalid
- LoRa parameters (frequency, bandwidth, sync word, baud rate) are fully configurable via the captive portal
- Default: 865.375 MHz, 250 kHz bandwidth, sync `12`, 115200 baud

### Navigation Display

- Live bearing arrow pointing from finder GPS position toward the CanSat
- When the finder is moving (speed > 1 m/s) and GPS course is valid, arrow shows **relative bearing** (CanSat direction relative to heading)
- When stationary, arrow shows **absolute bearing** (compass direction)
- Distance display in meters (< 1 km) or kilometers (>= 1 km)
- Circle compass with solar panel decorations around the arrow

### Proximity Buzzer

- Beep interval scales with distance: 500 ms at <= 30 m, 10 s at >= 3000 m
- Beeping only active when both GPS fix and valid CanSat data are present

### SD Card Logging

- Logs telemetry to `/logs/telemetry_N.csv` (auto-incremented filename)
- CSV columns: `Time, CS_Lat, CS_Lon, CS_Alt, CS_Speed, CS_Valid, GP_Lat, GP_Lon, GP_Alt, GP_Speed, GP_Sats, GP_Fix, Distance, Bearing`
- Write interval: 1 second
- SD uses HSPI to avoid conflicts with the display SPI bus

### Captive Portal Configuration

- On boot, opens a WiFi AP (`CanSat-Finder`, password `seatfinder`) for 3 minutes
- Configuration page served at `4.3.2.1`
- Configurable parameters: LoRa frequency, bandwidth, sync word, baud rate, CanSat name
- Settings persisted to NVS (`Preferences`) and survive power cycles

---

## Application States

```
STATE_TITLE        (3 s splash screen)
    |
STATE_CONFIG       (captive portal AP, 3 min timeout or on form submit)
    |
STATE_INITIALIZING (init GPS + LoRa)
    |
STATE_RUNNING      (main tracking loop)
```

---

## Telemetry Format

CanSat must transmit ASCII strings over LoRa in the following format:

```
latitude;longitude;altitude;speed
```

Example:
```
47.123456;18.654321;312.5;2.3
```

Fields:
- `latitude` - decimal degrees
- `longitude` - decimal degrees
- `altitude` - meters (optional, defaults to 0)
- `speed` - m/s (optional, defaults to 0)

---

## Display Layout

```
+--------------------------------------------------+
| [CanSat Name] Finder                             |
+--------------------------------------------------+
| CanSat Data:                  [  bearing arrow  ]|
|   Lat:  XX.XXXXXX             [  compass circle ]|
|   Lon:  XX.XXXXXX             [                 ]|
|   Alt:  XXXX.X m              Distance: XXXX m   |
|   Speed: X.X m/s                                 |
+--------------------------------------------------+
| Local Data:                                      |
|   Lat:  XX.XXXXXX                                |
|   Lon:  XX.XXXXXX                                |
|   Alt:  XXXX.X m                                 |
|   Speed: X.X m/s                                 |
|   Sat:  X                                        |
+--------------------------------------------------+
| SD: OK                                           |
+--------------------------------------------------+
```

Data values are color-coded: green when valid/fix acquired, red when invalid/no fix.

---

## GPS Handling

Uses the `BetterGPS` wrapper around TinyGPSPlus. Hungarian local time (CET/CEST) is calculated internally with DST boundary detection, though time is not displayed in this application — it is available for telemetry timestamping.

---

## Dependencies

- `TFT_eSPI`
- `TinyGPSPlus`
- `SD` (built-in ESP32 Arduino)
- `Preferences` (built-in)
- `WiFi`, `WebServer`, `DNSServer` (built-in)
- `math.h` (built-in)

---

## Notes

- The `BetterCapportal` class handles all captive portal redirect endpoints for Android, iOS, Windows, and Firefox detection
- LoRa module communicates via AT-style commands (`radio set freq`, `radio rx 0`, etc.) using the RN2483 command interface; received data arrives as `radio_rx <hex>` lines which are decoded from hex to ASCII
- Display selective redraw: only changed values trigger screen updates to minimize flicker on the 480x320 panel
