#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include "skytime_types.h"

void init_config();
bool load_config_files();
bool load_network_config();
bool load_system_config();
bool create_default_config_files();
const char *config_state_text(ConfigState state);

bool save_system_config_file();
bool save_network_config_file();
void apply_network_config_to_runtime();

void handle_web_config_page();
void handle_api_config_identity_get();
void handle_api_config_identity_post();
void handle_api_config_display_get();
void handle_api_config_display_post();
void handle_api_config_network_get();
void handle_api_config_network_post();
