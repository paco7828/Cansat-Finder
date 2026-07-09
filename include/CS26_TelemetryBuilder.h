#pragma once
// ============================================================================
//  CS26_TelemetryBuilder.h – SEAT_CS26_Telemetry lib v1.0.0
// ============================================================================
//  CanSat (TX) oldal: TelemetryData → CSV string
//
//  A globális sensorStore-t NEM olvassa közvetlenül.
//  A hívó (Telemetry.h) adja át a const TelemetryData& referenciát.
//  Így a lib önállóan fordítható, nincs globális függősége.
//
//  CSV formátum, ; elválasztóval, pozíció alapú parsing:
//    01  type;seq;ts;pcb_alive_hex;pow_hex;batt_v
//    02  type;seq;ts;lat;lon;alt_gps;alt_baro;speed;course;sats;gps_age_ms
//    03  type;seq;ts; BME680(5) ; BME280(4) ; BNO086(7) ; INA3221(6) ; VEML(1)
//    10  type;seq;ts;status_hex;cmd_seq;result_hex;cmd_str
// ============================================================================

#include <Arduino.h>
#include "CS26_TelemetryData.h"

namespace CS26Telem
{
  // ── Belső helper (NaN-biztos float→string) ─────────────────────────────────
  namespace _detail
  {
    inline int _fmt(char *buf, int len, float v, int dec = 1)
    {
      if (isnan(v))
        return snprintf(buf, len, "?");
      char fmt[10];
      snprintf(fmt, sizeof(fmt), "%%.%df", dec);
      return snprintf(buf, len, fmt, v);
    }
  } // namespace _detail

  // ── 01 – Status ────────────────────────────────────────────────────────────
  // type;seq;ts;pcb_alive_hex;pow_hex;batt_v
  // alive/pow maszkot a hívó (Telemetry.h) számítja ki a sensorStore alapján
  inline String buildStatus(uint16_t seq, uint32_t ts,
                            uint8_t alive_mask, uint8_t pow_mask,
                            float batt_v)
  {
    char buf[96], batt[10];
    _detail::_fmt(batt, sizeof(batt), batt_v, 2);
    snprintf(buf, sizeof(buf), "01;%u;%lu;%02X;%02X;%s",
             seq, (unsigned long)ts, alive_mask, pow_mask, batt);
    return String(buf);
  }

  // ── 02 – Avionics / GPS ────────────────────────────────────────────────────
  // type;seq;ts;lat;lon;alt_gps;alt_baro;speed;course;sats;gps_age_ms;hour;min;sec
  inline String buildAvionics(uint16_t seq, uint32_t ts,
                              const TelemetryData &s)
  {
    char buf[160]; // Megnövelve 140-ről 160-ra
    char lat[12], lon[12], ag[10], ab[10], spd[8], crs[8], age[8];

    _detail::_fmt(lat, sizeof(lat), s.gps_valid ? s.gps_lat : NAN, 5);
    _detail::_fmt(lon, sizeof(lon), s.gps_valid ? s.gps_lon : NAN, 5);
    _detail::_fmt(ag, sizeof(ag), s.gps_valid ? s.gps_alt_m : NAN, 0);
    _detail::_fmt(ab, sizeof(ab), s.bme680_alt_m, 0);
    _detail::_fmt(spd, sizeof(spd), s.gps_valid ? s.gps_speed : NAN, 1);
    _detail::_fmt(crs, sizeof(crs), s.gps_valid ? s.gps_course : NAN, 0);
    _detail::_fmt(age, sizeof(age), s.gps_valid ? (float)s.gps_age_ms : NAN, 0);

    // A végére odarakva a ;%02u;%02u;%02u és a három struktúra tag
    snprintf(buf, sizeof(buf), "02;%u;%lu;%s;%s;%s;%s;%s;%s;%u;%s;%02u;%02u;%02u",
             seq, (unsigned long)ts,
             lat, lon, ag, ab, spd, crs, s.gps_sats, age,
             s.gps_hour, s.gps_min, s.gps_sec);
    return String(buf);
  }

  // ── 03 – Sensor full ───────────────────────────────────────────────────────
  // type;seq;ts; BME680(5) ; BME280(4) ; BNO086(7) ; INA3221(6) ; VEML(1)
  inline String buildSensor(uint16_t seq, uint32_t ts,
                            const TelemetryData &s)
  {
    using namespace _detail;
    char buf[240], tmp[14];
    int n = snprintf(buf, sizeof(buf), "03;%u;%lu;", seq, (unsigned long)ts);

    auto a = [&](float v, int dec, bool semi = true)
    {
      _fmt(tmp, sizeof(tmp), v, dec);
      n += snprintf(buf + n, sizeof(buf) - n, "%s%s", tmp, semi ? ";" : "");
    };

    // BME680
    a(s.bme680_temp, 1);
    a(s.bme680_hum, 0);
    a(s.bme680_press, 1);
    a(s.bme680_alt_m, 0);
    a(s.bme680_gas_kohm, 0);
    // BME280
    a(s.bme280_temp, 1);
    a(s.bme280_hum, 0);
    a(s.bme280_press, 1);
    a(s.bme280_alt_m, 0);
    // BNO086
    a(s.imu_roll, 0);
    a(s.imu_pitch, 0);
    a(s.imu_yaw, 0);
    a(s.imu_linAcc, 2);
    a(s.imu_gyroX, 2);
    a(s.imu_gyroY, 2);
    a(s.imu_gyroZ, 2);
    // INA3221
    a(s.ina_ch1_v, 2);
    a(s.ina_ch1_a, 0);
    a(s.ina_ch2_v, 2);
    a(s.ina_ch2_a, 0);
    a(s.ina_ch3_v, 2);
    a(s.ina_ch3_a, 0);
    // VEML7700 (trailing ; nélkül)
    a(s.veml_lux, 0, false);

    return String(buf);
  }

  // ── 10 – CMD ACK ───────────────────────────────────────────────────────────
  // type;seq;ts;status_hex;cmd_seq;result_hex;cmd_str
  inline String buildCmdAck(uint16_t seq, uint32_t ts,
                            uint8_t status, uint8_t cmd_seq,
                            uint8_t result, const String &cmd)
  {
    char buf[80];
    String s = cmd.substring(0, 20);
    snprintf(buf, sizeof(buf), "10;%u;%lu;%02X;%u;%02X;%s",
             seq, (unsigned long)ts,
             status, cmd_seq, result, s.c_str());
    return String(buf);
  }

} // namespace CS26Telem
