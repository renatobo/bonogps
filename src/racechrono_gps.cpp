/******************************************************************************

  RaceChrono DIY BLE GPS encoder - see racechrono_gps.h

******************************************************************************/
#include "racechrono_gps.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// If both GGA and RMC were seen within this window, a fix is only sent once
// both sentences of the same epoch arrived. Otherwise (only one of them is
// enabled on the GPS) each sentence is sent on its own.
#define RC_PAIR_WINDOW_MS 2000UL
// VDOP only comes from GSA, which the RaceChrono preset polls every 5 s
#define RC_VDOP_MAX_AGE_MS 15000UL

#define RC_INVALID_LATLON 0x7FFFFFFF
#define RC_INVALID_U16 0xFFFF
#define RC_INVALID_U8 0xFF
#define RC_INVALID_SATS 0x3F

#define KNOTS_TO_KMH 1.852f

namespace
{

int hexval(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return -1;
}

bool isdigit_c(char c) { return c >= '0' && c <= '9'; }

int two_digits(const char *s) { return (s[0] - '0') * 10 + (s[1] - '0'); }

// "hhmmss" or "hhmmss.sss" -> ms since midnight
bool parse_time(const char *s, int32_t *out)
{
  for (int i = 0; i < 6; i++)
    if (!isdigit_c(s[i]))
      return false;
  int hh = two_digits(s), mm = two_digits(s + 2), ss = two_digits(s + 4);
  if (hh > 23 || mm > 59 || ss > 60)
    return false;
  int ms = 0;
  if (s[6] == '.')
  {
    // first 3 fractional digits, padded: ".2" -> 200, ".04" -> 40
    int scale = 100;
    for (const char *p = s + 7; isdigit_c(*p) && scale > 0; p++, scale /= 10)
      ms += (*p - '0') * scale;
  }
  *out = ((hh * 60 + mm) * 60 + ss) * 1000 + ms;
  return true;
}

// "ddmmyy" -> day, month, two-digit year
bool parse_date(const char *s, uint8_t *dd, uint8_t *mm, uint8_t *yy)
{
  for (int i = 0; i < 6; i++)
    if (!isdigit_c(s[i]))
      return false;
  int d = two_digits(s), m = two_digits(s + 2), y = two_digits(s + 4);
  if (d < 1 || d > 31 || m < 1 || m > 12)
    return false;
  *dd = d;
  *mm = m;
  *yy = y;
  return true;
}

bool parse_float(const char *s, float *out)
{
  if (!s[0])
    return false;
  char *end;
  double v = strtod(s, &end);
  if (end == s)
    return false;
  *out = (float)v;
  return true;
}

// NMEA "ddmm.mmmm" / "dddmm.mmmm" + hemisphere -> degrees * 1e7.
// Done in double: float's 24-bit mantissa cannot hold 1e-7 degree resolution.
bool parse_latlon(const char *v, const char *hemi, int32_t *out)
{
  if (!v[0] || !hemi[0])
    return false;
  char *end;
  double raw = strtod(v, &end);
  if (end == v)
    return false;
  double deg = floor(raw / 100.0);
  double minutes = raw - deg * 100.0;
  double value = (deg + minutes / 60.0) * 1e7;
  if (hemi[0] == 'S' || hemi[0] == 'W')
    value = -value;
  else if (hemi[0] != 'N' && hemi[0] != 'E')
    return false;
  *out = (int32_t)llround(value);
  return true;
}

uint16_t encode_altitude(float meters)
{
  // Fine encoding is masked to 15 bits: ceiling is 0x7FFF / 10 - 500 = 2776.7 m
  if (meters > 2776.7f)
  {
    long v = lroundf(meters + 500.0f);
    if (v > 0x7FFF)
      v = 0x7FFF;
    return (uint16_t)(v | 0x8000);
  }
  long v = lroundf((meters + 500.0f) * 10.0f);
  if (v < 0)
    v = 0;
  if (v > 0x7FFF)
    v = 0x7FFF;
  return (uint16_t)v;
}

uint16_t encode_speed(float kmh)
{
  if (kmh < 0)
    kmh = 0;
  // Fine encoding is masked to 15 bits: ceiling is 0x7FFF / 100 = 327.67 km/h
  if (kmh > 327.67f)
  {
    long v = lroundf(kmh * 10.0f);
    if (v > 0x7FFF)
      v = 0x7FFF;
    return (uint16_t)(v | 0x8000);
  }
  long v = lroundf(kmh * 100.0f);
  if (v > 0x7FFF)
    v = 0x7FFF;
  return (uint16_t)v;
}

uint8_t encode_dop(float dop)
{
  long v = lroundf(dop * 10.0f);
  if (v < 0)
    v = 0;
  if (v > 0xFE) // 0xFF means invalid
    v = 0xFE;
  return (uint8_t)v;
}

void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = v >> 8;
  p[1] = v;
}

void put_u32(uint8_t *p, uint32_t v)
{
  p[0] = v >> 24;
  p[1] = v >> 16;
  p[2] = v >> 8;
  p[3] = v;
}

} // namespace

void RaceChronoGpsEncoder::reset()
{
  line_len = 0;
  in_sentence = false;
  line_overflow = false;
  ready = false;
  memset(&ep, 0, sizeof(ep));
  ep.time_ms = -1;
  date_valid = false;
  date_yy = date_mm = date_dd = 0;
  vdop_valid = false;
  vdop = 0;
  vdop_ms = 0;
  now = 0;
  last_gga_ms = last_rmc_ms = 0;
  seen_gga = seen_rmc = false;
  prev_date_hour = -1;
  sync_bits = 0;
  time_changed = false;
  memset(main_packet, 0, sizeof(main_packet));
  memset(time_packet, 0, sizeof(time_packet));
  packets_built = 0;
  checksum_errors = 0;
}

bool RaceChronoGpsEncoder::feed(char c, uint32_t now_ms)
{
  if (c == '$')
  {
    // a '$' always starts a new sentence, even if the previous one was cut
    in_sentence = true;
    line_overflow = false;
    line_len = 0;
    line[line_len++] = c;
    return false;
  }
  if (!in_sentence)
    return false;
  if (c == '\r' || c == '\n')
  {
    in_sentence = false;
    if (line_overflow)
      return false;
    line[line_len] = '\0';
    now = now_ms;
    ready = false;
    processLine();
    return ready;
  }
  if (line_len < kLineMax - 1)
    line[line_len++] = c;
  else
    line_overflow = true;
  return false;
}

void RaceChronoGpsEncoder::processLine()
{
  // $ttsss,....*hh  - verify checksum (XOR of everything between $ and *)
  char *star = strchr(line, '*');
  if (!star || star[1] == '\0' || star[2] == '\0')
    return;
  uint8_t sum = 0;
  for (char *p = line + 1; p < star; p++)
    sum ^= (uint8_t)*p;
  int hi = hexval(star[1]), lo = hexval(star[2]);
  if (hi < 0 || lo < 0 || sum != (uint8_t)((hi << 4) | lo))
  {
    checksum_errors++;
    return;
  }
  *star = '\0';

  // split fields in place; f[0] is the address field, e.g. "GPGGA"
  char *f[kFieldsMax];
  int n = 0;
  char *p = line + 1;
  f[n++] = p;
  while (*p && n < kFieldsMax)
  {
    if (*p == ',')
    {
      *p = '\0';
      f[n++] = p + 1;
    }
    p++;
  }
  if (strlen(f[0]) != 5)
    return;
  // any talker (GP, GN, GL, ...): match on the sentence formatter only
  const char *type = f[0] + 2;
  if (strcmp(type, "GGA") == 0)
    handleGGA(f, n);
  else if (strcmp(type, "RMC") == 0)
    handleRMC(f, n);
  else if (strcmp(type, "GSA") == 0)
    handleGSA(f, n);
}

void RaceChronoGpsEncoder::beginEpoch(int32_t time_ms)
{
  if (time_ms == ep.time_ms)
    return;
  memset(&ep, 0, sizeof(ep));
  ep.time_ms = time_ms;
}

// $xxGGA,time,lat,N,lon,E,quality,sats,hdop,alt,M,geoid,M,age,station
void RaceChronoGpsEncoder::handleGGA(char **f, int n)
{
  if (n < 10)
    return;
  int32_t t;
  if (!parse_time(f[1], &t))
    return;
  beginEpoch(t);
  seen_gga = true;
  last_gga_ms = now;
  ep.has_gga = true;
  ep.gga_pos_valid = parse_latlon(f[2], f[3], &ep.gga_lat_e7) && parse_latlon(f[4], f[5], &ep.gga_lon_e7);
  ep.fix_quality = f[6][0] ? (uint8_t)atoi(f[6]) : 0;
  int sats = f[7][0] ? atoi(f[7]) : -1;
  ep.satellites = sats < 0 ? RC_INVALID_SATS : (sats > 0x3E ? 0x3E : (uint8_t)sats);
  ep.hdop_valid = parse_float(f[8], &ep.hdop);
  ep.alt_valid = parse_float(f[9], &ep.altitude_m);
  maybeEmit();
}

// $xxRMC,time,status,lat,N,lon,E,speed_knots,course,ddmmyy,magvar,E,mode[,navstatus]
void RaceChronoGpsEncoder::handleRMC(char **f, int n)
{
  if (n < 10)
    return;
  int32_t t;
  if (!parse_time(f[1], &t))
    return;
  // RMC is the only source of the date
  uint8_t dd, mm, yy;
  if (parse_date(f[9], &dd, &mm, &yy))
  {
    date_dd = dd;
    date_mm = mm;
    date_yy = yy;
    date_valid = true;
  }
  beginEpoch(t);
  seen_rmc = true;
  last_rmc_ms = now;
  ep.has_rmc = true;
  ep.rmc_active = (f[2][0] == 'A');
  ep.rmc_pos_valid = parse_latlon(f[3], f[4], &ep.rmc_lat_e7) && parse_latlon(f[5], f[6], &ep.rmc_lon_e7);
  float knots;
  ep.speed_valid = parse_float(f[7], &knots);
  if (ep.speed_valid)
    ep.speed_kmh = knots * KNOTS_TO_KMH;
  ep.course_valid = parse_float(f[8], &ep.course_deg);
  maybeEmit();
}

// $xxGSA,mode,fix,sv1..sv12,PDOP,HDOP,VDOP[,systemId]
void RaceChronoGpsEncoder::handleGSA(char **f, int n)
{
  if (n < 18)
    return;
  float v;
  if (parse_float(f[17], &v))
  {
    vdop = v;
    vdop_ms = now;
    vdop_valid = true;
  }
}

void RaceChronoGpsEncoder::maybeEmit()
{
  // the time characteristic needs the date, which only RMC carries
  if (ep.emitted || !date_valid)
    return;
  // Wait for the other sentence of this epoch only if the GPS is sending it
  bool need_gga = seen_gga && (now - last_gga_ms) <= RC_PAIR_WINDOW_MS;
  bool need_rmc = seen_rmc && (now - last_rmc_ms) <= RC_PAIR_WINDOW_MS;
  if ((need_gga && !ep.has_gga) || (need_rmc && !ep.has_rmc))
    return;
  build();
  ep.emitted = true;
  ready = true;
}

void RaceChronoGpsEncoder::build()
{
  int32_t t = ep.time_ms;
  int hour = t / 3600000;
  int minute = (t / 60000) % 60;
  int second = (t / 1000) % 60;
  int msec = t % 1000;

  // Time characteristic: hour and date, sync bits bump whenever it changes
  int32_t date_hour = (int32_t)date_yy * 8928 + (date_mm - 1) * 744 + (date_dd - 1) * 24 + hour;
  if (date_hour != prev_date_hour)
  {
    prev_date_hour = date_hour;
    sync_bits = (sync_bits + 1) & 0x7;
    time_packet[0] = (sync_bits << 5) | ((date_hour >> 16) & 0x1F);
    time_packet[1] = date_hour >> 8;
    time_packet[2] = date_hour;
    time_changed = true;
  }

  uint32_t since_hour = (uint32_t)minute * 30000 + (uint32_t)second * 500 + (uint32_t)msec / 2;
  main_packet[0] = (sync_bits << 5) | ((since_hour >> 16) & 0x1F);
  main_packet[1] = since_hour >> 8;
  main_packet[2] = since_hour;

  uint8_t quality;
  uint8_t sats;
  if (ep.has_gga)
  {
    quality = ep.fix_quality > 3 ? 3 : ep.fix_quality;
    sats = ep.satellites;
  }
  else
  {
    quality = ep.rmc_active ? 1 : 0;
    sats = RC_INVALID_SATS;
  }
  main_packet[3] = (quality << 6) | (sats & 0x3F);

  int32_t lat = RC_INVALID_LATLON, lon = RC_INVALID_LATLON;
  if (ep.gga_pos_valid)
  {
    lat = ep.gga_lat_e7;
    lon = ep.gga_lon_e7;
  }
  else if (ep.rmc_pos_valid)
  {
    lat = ep.rmc_lat_e7;
    lon = ep.rmc_lon_e7;
  }
  put_u32(main_packet + 4, (uint32_t)lat);
  put_u32(main_packet + 8, (uint32_t)lon);

  put_u16(main_packet + 12, ep.alt_valid ? encode_altitude(ep.altitude_m) : RC_INVALID_U16);
  put_u16(main_packet + 14, ep.speed_valid ? encode_speed(ep.speed_kmh) : RC_INVALID_U16);

  uint16_t bearing = RC_INVALID_U16;
  if (ep.course_valid)
  {
    long b = lroundf(ep.course_deg * 100.0f) % 36000;
    if (b < 0)
      b += 36000;
    bearing = (uint16_t)b;
  }
  put_u16(main_packet + 16, bearing);

  main_packet[18] = ep.hdop_valid ? encode_dop(ep.hdop) : RC_INVALID_U8;
  bool vdop_fresh = vdop_valid && (now - vdop_ms) <= RC_VDOP_MAX_AGE_MS;
  main_packet[19] = vdop_fresh ? encode_dop(vdop) : RC_INVALID_U8;

  packets_built++;
}
