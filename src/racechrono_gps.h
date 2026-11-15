/******************************************************************************

  RaceChrono DIY BLE GPS encoder

  Turns the NMEA stream coming from the GPS (GGA + RMC, plus GSA for VDOP)
  into the binary packets of the RaceChrono BLE DIY API:
    - GPS main characteristic 0x0003: 20 bytes, one per fix
    - GPS time characteristic 0x0004:  3 bytes, changes once per hour
  Spec: https://github.com/aollin/racechrono-ble-diy-device

  One 20-byte notification per fix (vs. ~150 bytes of NMEA text) is what makes
  25 Hz comfortable over BLE.

  Plain C++ with no Arduino or BLE dependency, so it can be built and tested
  on a host machine.

******************************************************************************/
#pragma once

#include <stdint.h>
#include <stddef.h>

#define RACECHRONO_GPS_MAIN_SIZE 20
#define RACECHRONO_GPS_TIME_SIZE 3

class RaceChronoGpsEncoder
{
public:
  RaceChronoGpsEncoder() { reset(); }

  void reset();

  // Feed one byte of the GPS UART stream. now_ms is a monotonic clock
  // (millis()) used only to age out the VDOP value from polled GSA sentences.
  // Returns true when a new main packet is ready in mainPacket().
  bool feed(char c, uint32_t now_ms);

  // 20-byte payload for characteristic 0x0003
  const uint8_t *mainPacket() const { return main_packet; }
  // 3-byte payload for characteristic 0x0004
  const uint8_t *timePacket() const { return time_packet; }

  // True once after the time packet (date/hour, and with it the sync bits)
  // changed. Push the new time value to 0x0004 BEFORE notifying 0x0003, so a
  // client never sees main packets whose sync bits it cannot match.
  bool takeTimeChanged()
  {
    bool changed = time_changed;
    time_changed = false;
    return changed;
  }

  // Counters, for logging/diagnostics
  uint32_t packetsBuilt() const { return packets_built; }
  uint32_t checksumErrors() const { return checksum_errors; }

private:
  static const size_t kLineMax = 128;
  static const int kFieldsMax = 24;

  struct Epoch
  {
    int32_t time_ms; // ms since 00:00 UTC, -1 = no epoch
    bool has_gga, has_rmc, emitted;
    // GGA
    bool gga_pos_valid;
    int32_t gga_lat_e7, gga_lon_e7;
    uint8_t fix_quality, satellites;
    bool hdop_valid;
    float hdop;
    bool alt_valid;
    float altitude_m;
    // RMC
    bool rmc_pos_valid;
    int32_t rmc_lat_e7, rmc_lon_e7;
    bool rmc_active; // status 'A'
    bool speed_valid;
    float speed_kmh;
    bool course_valid;
    float course_deg;
  };

  void processLine();
  void handleGGA(char **f, int n);
  void handleRMC(char **f, int n);
  void handleGSA(char **f, int n);
  // Called with the time of a GGA/RMC sentence before merging it: a new time
  // starts a new epoch (an incomplete previous epoch is dropped).
  void beginEpoch(int32_t time_ms);
  void maybeEmit();
  void build();

  char line[kLineMax];
  size_t line_len;
  bool in_sentence;
  bool line_overflow;
  bool ready; // a packet was built while processing the current line

  Epoch ep;

  bool date_valid;
  uint8_t date_yy, date_mm, date_dd;

  bool vdop_valid;
  float vdop;
  uint32_t vdop_ms;
  uint32_t now;

  // which of GGA/RMC the GPS is currently sending
  bool seen_gga, seen_rmc;
  uint32_t last_gga_ms, last_rmc_ms;

  int32_t prev_date_hour;
  uint8_t sync_bits;
  bool time_changed;

  uint8_t main_packet[RACECHRONO_GPS_MAIN_SIZE];
  uint8_t time_packet[RACECHRONO_GPS_TIME_SIZE];

  uint32_t packets_built;
  uint32_t checksum_errors;
};
