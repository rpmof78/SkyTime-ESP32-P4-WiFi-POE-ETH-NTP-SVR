#include <Arduino.h>
#include <TinyGPS++.h>
#include <WebServer.h>

#include "skytime_types.h"
#include "gnss_manager.h"

#define GPS_STALE_MS             3000UL
#define GPS_HOLDOVER_MAX_MS      300000UL
#define GPS_LOCK_MIN_SATS        4

char gnss_nmea_line[GNSS_NMEA_LINE_MAX] = {};
uint8_t gnss_nmea_length = 0;

uint16_t gnss_used_gps[GNSS_MAX_USED_PRNS] = {};
uint8_t gnss_used_gps_count = 0;
uint16_t gnss_used_bds[GNSS_MAX_USED_PRNS] = {};
uint8_t gnss_used_bds_count = 0;

extern TinyGPSPlus gps;
extern GPSData gps_data;
extern PPSData pps_data;
extern TimingData timing_data;
extern GnssRuntime gnss_runtime;
extern portMUX_TYPE gnss_mux;
extern portMUX_TYPE timing_mux;

extern WebServer web_server;
extern WebRuntime web_runtime;

extern bool append_json_uint(
  String &output,
  uint64_t value
);
extern void json_escape_string(
  const char *src,
  char *dst,
  size_t dst_size
);

static GnssConstellation gnss_constellation_from_gsv(
  const char *sentence_id
);
static GnssConstellation gnss_constellation_from_system_id(
  int system_id
);
static bool gnss_prn_in_list(
  uint16_t prn,
  const uint16_t *list,
  uint8_t count
);
static bool gnss_prn_used(
  GnssConstellation constellation,
  uint16_t prn
);
static void gnss_clear_constellation(
  GnssConstellation constellation
);
static GnssSatellite *gnss_find_or_add_satellite(
  GnssConstellation constellation,
  uint16_t prn
);
static bool gnss_checksum_valid(
  const char *sentence
);
static uint8_t gnss_split_fields(
  char *sentence,
  char **fields,
  uint8_t max_fields
);
static void gnss_update_used_flags(
  GnssConstellation constellation
);
static void gnss_parse_gsa(
  char **fields,
  uint8_t field_count
);
static void gnss_parse_gsv(
  char **fields,
  uint8_t field_count
);
static void gnss_parse_txt(
  char **fields,
  uint8_t field_count
);

const char *gnss_constellation_text(GnssConstellation constellation) {
  switch (constellation) {
    case GNSS_CONSTELLATION_GPS: return "GPS";
    case GNSS_CONSTELLATION_BDS: return "BDS";
    default: return "UNKNOWN";
  }
}

static GnssConstellation gnss_constellation_from_gsv(const char *sentence_id) {
  if (!sentence_id) return GNSS_CONSTELLATION_UNKNOWN;
  if (strncmp(sentence_id, "$GPGSV", 6) == 0) return GNSS_CONSTELLATION_GPS;
  if (strncmp(sentence_id, "$BDGSV", 6) == 0) return GNSS_CONSTELLATION_BDS;
  return GNSS_CONSTELLATION_UNKNOWN;
}

static GnssConstellation gnss_constellation_from_system_id(int system_id) {
  if (system_id == 1) return GNSS_CONSTELLATION_GPS;
  if (system_id == 4) return GNSS_CONSTELLATION_BDS;
  return GNSS_CONSTELLATION_UNKNOWN;
}

static bool gnss_prn_in_list(
  uint16_t prn,
  const uint16_t *list,
  uint8_t count
) {
  for (uint8_t i = 0; i < count; i++) {
    if (list[i] == prn) return true;
  }
  return false;
}

static bool gnss_prn_used(GnssConstellation constellation, uint16_t prn) {
  if (constellation == GNSS_CONSTELLATION_GPS) {
    return gnss_prn_in_list(prn, gnss_used_gps, gnss_used_gps_count);
  }

  if (constellation == GNSS_CONSTELLATION_BDS) {
    return gnss_prn_in_list(prn, gnss_used_bds, gnss_used_bds_count);
  }

  return false;
}

static void gnss_clear_constellation(GnssConstellation constellation) {
  uint8_t write_index = 0;

  for (uint8_t i = 0; i < gnss_runtime.satellite_count; i++) {
    if (gnss_runtime.satellites[i].constellation != constellation) {
      if (write_index != i) {
        gnss_runtime.satellites[write_index] = gnss_runtime.satellites[i];
      }
      write_index++;
    }
  }

  gnss_runtime.satellite_count = write_index;
}

static GnssSatellite *gnss_find_or_add_satellite(
  GnssConstellation constellation,
  uint16_t prn
) {
  for (uint8_t i = 0; i < gnss_runtime.satellite_count; i++) {
    GnssSatellite &satellite = gnss_runtime.satellites[i];

    if (satellite.constellation == constellation && satellite.prn == prn) {
      return &satellite;
    }
  }

  if (gnss_runtime.satellite_count >= GNSS_MAX_SATELLITES) {
    return nullptr;
  }

  GnssSatellite &satellite =
    gnss_runtime.satellites[gnss_runtime.satellite_count++];

  satellite = {};
  satellite.constellation = constellation;
  satellite.prn = prn;
  return &satellite;
}

static bool gnss_checksum_valid(const char *sentence) {
  if (!sentence || sentence[0] != '$') return false;

  const char *asterisk = strchr(sentence, '*');
  if (!asterisk || strlen(asterisk) < 3) return false;

  uint8_t calculated = 0;

  for (const char *p = sentence + 1; p < asterisk; p++) {
    calculated ^= (uint8_t)*p;
  }

  char checksum_text[3] = {asterisk[1], asterisk[2], '\0'};
  uint8_t expected = (uint8_t)strtoul(checksum_text, nullptr, 16);

  return calculated == expected;
}

static uint8_t gnss_split_fields(
  char *sentence,
  char **fields,
  uint8_t max_fields
) {
  uint8_t count = 0;
  char *cursor = sentence;

  while (cursor && count < max_fields) {
    fields[count++] = cursor;
    char *comma = strchr(cursor, ',');

    if (!comma) break;

    *comma = '\0';
    cursor = comma + 1;
  }

  return count;
}

static void gnss_update_used_flags(GnssConstellation constellation) {
  uint8_t used_count = 0;

  for (uint8_t i = 0; i < gnss_runtime.satellite_count; i++) {
    GnssSatellite &satellite = gnss_runtime.satellites[i];

    if (satellite.constellation == constellation) {
      satellite.used_in_fix = gnss_prn_used(
        satellite.constellation,
        satellite.prn
      );
    }

    if (satellite.used_in_fix) used_count++;
  }

  gnss_runtime.used_count = used_count;
}

static void gnss_parse_gsa(char **fields, uint8_t field_count) {
  if (field_count < 18) return;

  int fix_dimension = atoi(fields[2]);
  if (fix_dimension >= 1 && fix_dimension <= 3) {
    gnss_runtime.fix_dimension = (uint8_t)fix_dimension;
  }

  int system_id = atoi(fields[field_count - 1]);
  GnssConstellation constellation =
    gnss_constellation_from_system_id(system_id);

  if (constellation == GNSS_CONSTELLATION_UNKNOWN) return;

  uint16_t *used_list =
    constellation == GNSS_CONSTELLATION_GPS
      ? gnss_used_gps
      : gnss_used_bds;

  uint8_t *used_count =
    constellation == GNSS_CONSTELLATION_GPS
      ? &gnss_used_gps_count
      : &gnss_used_bds_count;

  *used_count = 0;

  for (uint8_t i = 3; i <= 14 && i < field_count; i++) {
    if (!fields[i] || fields[i][0] == '\0') continue;

    uint16_t prn = (uint16_t)atoi(fields[i]);

    if (prn > 0 && *used_count < GNSS_MAX_USED_PRNS) {
      used_list[(*used_count)++] = prn;
    }
  }

  gnss_runtime.last_gsa_ms = millis();
  gnss_update_used_flags(constellation);
}

static void gnss_parse_gsv(char **fields, uint8_t field_count) {
  if (field_count < 4) return;

  GnssConstellation constellation =
    gnss_constellation_from_gsv(fields[0]);

  if (constellation == GNSS_CONSTELLATION_UNKNOWN) return;

  int message_number = atoi(fields[2]);
  int visible = atoi(fields[3]);

  if (message_number == 1) {
    gnss_clear_constellation(constellation);

    if (constellation == GNSS_CONSTELLATION_GPS) {
      gnss_runtime.gps_visible = constrain(visible, 0, 255);
    } else if (constellation == GNSS_CONSTELLATION_BDS) {
      gnss_runtime.bds_visible = constrain(visible, 0, 255);
    }
  }

  uint32_t now_ms = millis();

  for (uint8_t base = 4; base + 3 < field_count; base += 4) {
    if (!fields[base] || fields[base][0] == '\0') continue;

    uint16_t prn = (uint16_t)atoi(fields[base]);
    if (prn == 0) continue;

    GnssSatellite *satellite =
      gnss_find_or_add_satellite(constellation, prn);

    if (!satellite) continue;

    satellite->elevation_deg =
      fields[base + 1][0] ? constrain(atoi(fields[base + 1]), 0, 90) : 0;

    satellite->azimuth_deg =
      fields[base + 2][0] ? constrain(atoi(fields[base + 2]), 0, 359) : 0;

    satellite->snr_valid = fields[base + 3][0] != '\0';
    satellite->snr_dbhz = satellite->snr_valid
      ? constrain(atoi(fields[base + 3]), 0, 99)
      : 0;

    satellite->used_in_fix = gnss_prn_used(constellation, prn);
    satellite->last_seen_ms = now_ms;
  }

  gnss_runtime.last_gsv_ms = now_ms;
  gnss_update_used_flags(constellation);
}

static void gnss_parse_txt(char **fields, uint8_t field_count) {
  if (field_count < 5 || !fields[4]) return;

  snprintf(
    gnss_runtime.antenna_status,
    sizeof(gnss_runtime.antenna_status),
    "%s",
    fields[4]
  );

  gnss_runtime.antenna_ok =
    strstr(fields[4], "ANTENNA OK") != nullptr;
}

void gnss_parse_nmea_sentence(char *sentence) {
  if (!sentence || sentence[0] != '$') return;

  if (!gnss_checksum_valid(sentence)) {
    portENTER_CRITICAL(&gnss_mux);
    gnss_runtime.checksum_errors++;
    portEXIT_CRITICAL(&gnss_mux);
    return;
  }

  char *asterisk = strchr(sentence, '*');
  if (asterisk) *asterisk = '\0';

  char *fields[24] = {};
  uint8_t field_count = gnss_split_fields(sentence, fields, 24);

  if (field_count == 0) return;

  portENTER_CRITICAL(&gnss_mux);

  if (strcmp(fields[0], "$GPGSV") == 0 ||
      strcmp(fields[0], "$BDGSV") == 0) {
    gnss_parse_gsv(fields, field_count);
  } else if (strcmp(fields[0], "$GNGSA") == 0) {
    gnss_parse_gsa(fields, field_count);
  } else if (strcmp(fields[0], "$GPTXT") == 0) {
    gnss_parse_txt(fields, field_count);
  }

  gnss_runtime.sentences_parsed++;
  portEXIT_CRITICAL(&gnss_mux);
}

void gnss_process_nmea_char(char c) {
  if (c == '\r') return;

  if (c == '\n') {
    if (gnss_nmea_length > 0) {
      gnss_nmea_line[gnss_nmea_length] = '\0';
      gnss_parse_nmea_sentence(gnss_nmea_line);
      gnss_nmea_length = 0;
    }

    return;
  }

  if (c == '$') {
    gnss_nmea_length = 0;
  }

  if (gnss_nmea_length < GNSS_NMEA_LINE_MAX - 1) {
    gnss_nmea_line[gnss_nmea_length++] = c;
  } else {
    gnss_nmea_length = 0;
  }
}

void process_gps() {
  bool sentence_updated = false;

  while (Serial2.available() > 0) {
    char c = Serial2.read();

    gnss_process_nmea_char(c);

    if (gps.encode(c)) {
      sentence_updated = true;
    }
  }

  if (!sentence_updated) {
    update_gps_health();
    return;
  }

  uint32_t now_ms = millis();
  gps_data.last_update_ms = now_ms;

  gps_data.time_valid = gps.time.isValid();
  gps_data.date_valid = gps.date.isValid();
  gps_data.location_valid = gps.location.isValid();

  if (gps.satellites.isValid()) {
    gps_data.satellites = gps.satellites.value();
  }

  if (gps.location.isValid()) {
    gps_data.latitude = gps.location.lat();
    gps_data.longitude = gps.location.lng();
    gps_data.last_location_update_ms = now_ms;
  }

  if (gps.altitude.isValid()) {
    gps_data.altitude = gps.altitude.meters();
  }

  if (gps.date.isValid()) {
    gps_data.year = gps.date.year();
    gps_data.month = gps.date.month();
    gps_data.day = gps.date.day();
  }

  if (gps.time.isValid()) {
    gps_data.hour = gps.time.hour();
    gps_data.minute = gps.time.minute();
    gps_data.second = gps.time.second();
    gps_data.last_time_update_ms = now_ms;
  }

  update_gps_health();
}

void update_gps_health() {
  uint32_t now_ms = millis();

  bool time_fresh =
    gps_data.last_time_update_ms > 0 &&
    (now_ms - gps_data.last_time_update_ms) <= GPS_STALE_MS;

  bool fix_good =
    gps_data.time_valid &&
    gps_data.date_valid &&
    gps_data.satellites >= GPS_LOCK_MIN_SATS &&
    time_fresh;

  if (fix_good) {
    gps_data.locked = true;
    gps_data.state = GPS_LOCKED;
    gps_data.last_locked_ms = now_ms;

    portENTER_CRITICAL(&timing_mux);
    timing_data.holdover = false;
    timing_data.holdover_start_ms = 0;
    portEXIT_CRITICAL(&timing_mux);
    return;
  }

  bool pps_usable =
    pps_data.edge_seen &&
    pps_data.current_pps_time_us > 0 &&
    pps_data.state != PPS_TIMEOUT;

  bool holdover_allowed =
    gps_data.last_locked_ms > 0 &&
    (now_ms - gps_data.last_locked_ms) <= GPS_HOLDOVER_MAX_MS &&
    pps_usable &&
    pps_data.last_pps_epoch > 0;

  if (holdover_allowed) {
    gps_data.locked = false;
    gps_data.state = GPS_HOLDOVER;

    portENTER_CRITICAL(&timing_mux);
    if (!timing_data.holdover) {
      timing_data.holdover = true;
      timing_data.holdover_start_ms = now_ms;
    }
    portEXIT_CRITICAL(&timing_mux);
  } else if (gps_data.last_update_ms > 0 &&
             (now_ms - gps_data.last_update_ms) > GPS_STALE_MS) {
    gps_data.locked = false;
    gps_data.state = GPS_STALE;
    portENTER_CRITICAL(&timing_mux);
    timing_data.holdover = false;
    portEXIT_CRITICAL(&timing_mux);
  } else {
    gps_data.locked = false;
    gps_data.state = GPS_SEARCHING;
    portENTER_CRITICAL(&timing_mux);
    timing_data.holdover = false;
    portEXIT_CRITICAL(&timing_mux);
  }
}

void handle_api_gnss() {
  web_runtime.requests_total++;
  web_runtime.requests_api++;
  snprintf(
    web_runtime.last_uri,
    sizeof(web_runtime.last_uri),
    "%s",
    web_server.uri().c_str()
  );

  GnssSatellite snapshot[GNSS_MAX_SATELLITES];
  uint8_t satellite_count = 0;
  uint8_t gps_visible = 0;
  uint8_t bds_visible = 0;
  uint8_t used_count = 0;
  uint8_t fix_dimension = 0;
  uint32_t last_gsv_ms = 0;
  uint32_t last_gsa_ms = 0;
  uint32_t sentences_parsed = 0;
  uint32_t checksum_errors = 0;
  bool antenna_ok = false;
  char antenna_status[24] = "";

  portENTER_CRITICAL(&gnss_mux);
  satellite_count = gnss_runtime.satellite_count;
  gps_visible = gnss_runtime.gps_visible;
  bds_visible = gnss_runtime.bds_visible;
  used_count = gnss_runtime.used_count;
  fix_dimension = gnss_runtime.fix_dimension;
  last_gsv_ms = gnss_runtime.last_gsv_ms;
  last_gsa_ms = gnss_runtime.last_gsa_ms;
  sentences_parsed = gnss_runtime.sentences_parsed;
  checksum_errors = gnss_runtime.checksum_errors;
  antenna_ok = gnss_runtime.antenna_ok;
  snprintf(
    antenna_status,
    sizeof(antenna_status),
    "%s",
    gnss_runtime.antenna_status
  );

  for (uint8_t i = 0; i < satellite_count; i++) {
    snapshot[i] = gnss_runtime.satellites[i];
  }
  portEXIT_CRITICAL(&gnss_mux);

  uint32_t now_ms = millis();
  uint32_t gsv_age_ms =
    last_gsv_ms > 0 ? now_ms - last_gsv_ms : 0;

  uint32_t gsa_age_ms =
    last_gsa_ms > 0 ? now_ms - last_gsa_ms : 0;

  String json;
  json.reserve(4096);

  json += "{\"ok\":true,\"gps_visible\":";
  append_json_uint(json, gps_visible);
  json += ",\"bds_visible\":";
  append_json_uint(json, bds_visible);
  json += ",\"visible_total\":";
  append_json_uint(
    json,
    (uint16_t)gps_visible + (uint16_t)bds_visible
  );
  json += ",\"used_total\":";
  append_json_uint(json, used_count);
  json += ",\"fix_dimension\":";
  append_json_uint(json, fix_dimension);
  json += ",\"gsv_age_ms\":";
  append_json_uint(json, gsv_age_ms);
  json += ",\"gsa_age_ms\":";
  append_json_uint(json, gsa_age_ms);
  json += ",\"antenna_ok\":";
  json += antenna_ok ? "true" : "false";
  json += ",\"antenna_status\":\"";

  char escaped_antenna_status[48];
  json_escape_string(
    antenna_status,
    escaped_antenna_status,
    sizeof(escaped_antenna_status)
  );

  json += escaped_antenna_status;
  json += "\",\"sentences_parsed\":";
  append_json_uint(json, sentences_parsed);
  json += ",\"checksum_errors\":";
  append_json_uint(json, checksum_errors);
  json += ",\"satellites\":[";

  for (uint8_t i = 0; i < satellite_count; i++) {
    const GnssSatellite &satellite = snapshot[i];

    if (i > 0) json += ",";

    json += "{\"constellation\":\"";
    json += gnss_constellation_text(satellite.constellation);
    json += "\",\"prn\":";
    append_json_uint(json, satellite.prn);
    json += ",\"elevation\":";
    append_json_uint(json, satellite.elevation_deg);
    json += ",\"azimuth\":";
    append_json_uint(json, satellite.azimuth_deg);
    json += ",\"snr\":";

    if (satellite.snr_valid) {
      append_json_uint(json, satellite.snr_dbhz);
    } else {
      json += "null";
    }

    json += ",\"used\":";
    json += satellite.used_in_fix ? "true" : "false";
    json += ",\"age_ms\":";
    append_json_uint(
      json,
      satellite.last_seen_ms > 0
        ? now_ms - satellite.last_seen_ms
        : 0
    );
    json += "}";
  }

  json += "]}";

  web_server.sendHeader("Cache-Control", "no-store");
  web_server.send(200, "application/json", json);
}
