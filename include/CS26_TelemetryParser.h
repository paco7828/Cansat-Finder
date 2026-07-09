#pragma once
// ============================================================================
//  CS26_TelemetryParser.h – SEAT_CS26_Telemetry lib v1.0.0
// ============================================================================
//  GND (RX) oldal: CSV string → TelemetryData struct (inkrementális írás)
//
//  Inkrementális design:
//    A parse* függvények AZ OUT STRUCT CSAK RELEVÁNS MEZŐIT frissítik,
//    a többit érintetlenül hagyják. GND oldalon egyetlen TelemetryData
//    gndStore példányban mindig a legfrissebb értékek vannak szenzortípusonként
//    – akárcsak a CanSat sensorStore-ban.
//
//  Tipikus GND használat:
//    TelemetryData gndStore;
//    if (CS26Telem::parseAny(receivedCsv, gndStore)) { ... }
// ============================================================================

#include <Arduino.h>
#include "CS26_TelemetryData.h"

namespace CS26Telem
{
  // ── Belső CSV helper ───────────────────────────────────────────────────────
  namespace _detail
  {
    /// CSV sort ';' mentén feldarabol, max maxFields darabra.
    /// Visszaadja a ténylegesen talált mezők számát.
    inline uint8_t splitCSV(const String &csv, String *fields, uint8_t maxFields)
    {
      uint8_t count = 0;
      int start = 0;
      for (int i = 0; i <= (int)csv.length() && count < maxFields; i++)
      {
        if (i == (int)csv.length() || csv[i] == ';')
        {
          fields[count++] = csv.substring(start, i);
          start = i + 1;
        }
      }
      return count;
    }

    /// "?" → NAN, egyébként toFloat()
    inline float parseF(const String &s)
    {
      if (s == "?")
        return NAN;
      return s.toFloat();
    }

    /// Hex string → uint8_t  (pl. "0A" → 10)
    inline uint8_t parseHex8(const String &s)
    {
      return (uint8_t)strtoul(s.c_str(), nullptr, 16);
    }
  } // namespace _detail

  // ── detectType ─────────────────────────────────────────────────────────────
  /// Visszaadja a csomag típusát (1, 2, 3, 10) a CSV első mezőjéből,
  /// vagy 0 ha üres / ismeretlen.
  inline uint8_t detectType(const String &csv)
  {
    int idx = csv.indexOf(';');
    String t = (idx < 0) ? csv : csv.substring(0, idx);
    return (uint8_t)t.toInt();
  }

  // ── 01 – Status ────────────────────────────────────────────────────────────
  // 01;seq;ts;alive_hex;pow_hex;batt_v
  // alive bitmask: bit0=MAIN, bit1=PDU, bit2=TOP, bit3=BOT
  // pow   bitmask: bit0=TOP,  bit1=BOT, bit2=CAM
  inline bool parseStatus(const String &csv, TelemetryData &out)
  {
    String f[6];
    if (_detail::splitCSV(csv, f, 6) < 6)
      return false;
    if (f[0].toInt() != 1)
      return false;

    out.last_rx_seq = (uint16_t)f[1].toInt();
    out.last_rx_ts = (uint32_t)f[2].toInt();
    out.last_rx_type = 1;

    uint8_t alive = _detail::parseHex8(f[3]);
    uint8_t pow = _detail::parseHex8(f[4]);

    out.pdu_alive = (alive & 0x02) != 0;
    out.top_alive = (alive & 0x04) != 0;
    out.bot_alive = (alive & 0x08) != 0;

    out.pdu_pow_top = (pow & 0x01) != 0;
    out.pdu_pow_bottom = (pow & 0x02) != 0;
    out.pdu_pow_camera = (pow & 0x04) != 0;

    // batt_v → ina_ch1_v (legjobb elérhető adat)
    out.ina_ch1_v = _detail::parseF(f[5]);

    return true;
  }

  // ── 02 – Avionics / GPS ────────────────────────────────────────────────────
  // 02;seq;ts;lat;lon;alt_gps;alt_baro;speed;course;sats;gps_age_ms;hour;min;sec
  inline bool parseAvionics(const String &csv, TelemetryData &out)
  {
    String f[14]; // 11-ről átírva 14-re
    if (_detail::splitCSV(csv, f, 14) < 14)
      return false; // Itt is 14
    if (f[0].toInt() != 2)
      return false;

    out.last_rx_seq = (uint16_t)f[1].toInt();
    out.last_rx_ts = (uint32_t)f[2].toInt();
    out.last_rx_type = 2;

    out.gps_lat = _detail::parseF(f[3]);
    out.gps_lon = _detail::parseF(f[4]);
    out.gps_alt_m = _detail::parseF(f[5]);
    out.bme680_alt_m = _detail::parseF(f[6]);
    out.gps_speed = _detail::parseF(f[7]);
    out.gps_course = _detail::parseF(f[8]);
    out.gps_sats = (uint8_t)f[9].toInt();

    float age = _detail::parseF(f[10]);
    out.gps_age_ms = isnan(age) ? 99999u : (uint32_t)age;

    // Új időmezők kiszedése a stringből
    out.gps_hour = (uint8_t)f[11].toInt();
    out.gps_min = (uint8_t)f[12].toInt();
    out.gps_sec = (uint8_t)f[13].toInt();

    out.gps_valid = !isnan(out.gps_lat) && !isnan(out.gps_lon);
    out.ts_gps = millis();

    return true;
  }

  // ── 03 – Sensor full ───────────────────────────────────────────────────────
  // 03;seq;ts; BME680(5) ; BME280(4) ; BNO086(7) ; INA3221(6) ; VEML(1)
  // Összesen: 3 fejléc + 23 adat = 26 mező
  inline bool parseSensor(const String &csv, TelemetryData &out)
  {
    String f[26];
    if (_detail::splitCSV(csv, f, 26) < 26)
      return false;
    if (f[0].toInt() != 3)
      return false;

    out.last_rx_seq = (uint16_t)f[1].toInt();
    out.last_rx_ts = (uint32_t)f[2].toInt();
    out.last_rx_type = 3;

    // BME680 [f3..f7]
    out.bme680_temp = _detail::parseF(f[3]);
    out.bme680_hum = _detail::parseF(f[4]);
    out.bme680_press = _detail::parseF(f[5]);
    out.bme680_alt_m = _detail::parseF(f[6]);
    out.bme680_gas_kohm = _detail::parseF(f[7]);
    out.ts_bme680 = millis();

    // BME280 [f8..f11]
    out.bme280_temp = _detail::parseF(f[8]);
    out.bme280_hum = _detail::parseF(f[9]);
    out.bme280_press = _detail::parseF(f[10]);
    out.bme280_alt_m = _detail::parseF(f[11]);
    out.ts_bme280 = millis();

    // BNO086 [f12..f18]
    out.imu_roll = _detail::parseF(f[12]);
    out.imu_pitch = _detail::parseF(f[13]);
    out.imu_yaw = _detail::parseF(f[14]);
    out.imu_linAcc = _detail::parseF(f[15]);
    out.imu_gyroX = _detail::parseF(f[16]);
    out.imu_gyroY = _detail::parseF(f[17]);
    out.imu_gyroZ = _detail::parseF(f[18]);
    out.ts_imu = millis();

    // INA3221 [f19..f24]
    out.ina_ch1_v = _detail::parseF(f[19]);
    out.ina_ch1_a = _detail::parseF(f[20]);
    out.ina_ch2_v = _detail::parseF(f[21]);
    out.ina_ch2_a = _detail::parseF(f[22]);
    out.ina_ch3_v = _detail::parseF(f[23]);
    out.ina_ch3_a = _detail::parseF(f[24]);
    out.ts_ina = millis();

    // VEML7700 [f25]
    out.veml_lux = _detail::parseF(f[25]);
    out.ts_veml = millis();

    return true;
  }

  // ── 10 – CMD ACK ───────────────────────────────────────────────────────────
  // 10;seq;ts;status_hex;cmd_seq;result_hex;cmd_str
  // A TelemetryData-ban nincs dedikált CMD ACK mező – csak last_rx_* frissül.
  // A GND kód a last_rx_type == 10 feltétellel kezelheti külön logikával.
  inline bool parseCmdAck(const String &csv, TelemetryData &out)
  {
    String f[7];
    if (_detail::splitCSV(csv, f, 7) < 6)
      return false;
    if (f[0].toInt() != 10)
      return false;

    out.last_rx_seq = (uint16_t)f[1].toInt();
    out.last_rx_ts = (uint32_t)f[2].toInt();
    out.last_rx_type = 10;

    return true;
  }

  // ── parseAny – kényelmi wrapper ────────────────────────────────────────────
  /// Típustól függően a megfelelő parse*-t hívja.
  /// true = sikeres parse, false = ismeretlen formátum / hiányzó mező
  inline bool parseAny(const String &csv, TelemetryData &out)
  {
    switch (detectType(csv))
    {
    case 1:
      return parseStatus(csv, out);
    case 2:
      return parseAvionics(csv, out);
    case 3:
      return parseSensor(csv, out);
    case 10:
      return parseCmdAck(csv, out);
    default:
      return false;
    }
  }

} // namespace CS26Telem
