#pragma once

#include <Arduino.h>
#include "skytime_types.h"

void process_gps();
void update_gps_health();

void gnss_process_nmea_char(char c);
void gnss_parse_nmea_sentence(char *sentence);
const char *gnss_constellation_text(
  GnssConstellation constellation
);

void handle_api_gnss();
