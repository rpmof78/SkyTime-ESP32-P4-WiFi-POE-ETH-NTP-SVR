#pragma once

#include <Arduino.h>
#include "skytime_types.h"

void init_web_server();
void update_web_server();
const char *web_state_text(WebState state);

bool send_sd_file(
  const char *path,
  const char *content_type
);
