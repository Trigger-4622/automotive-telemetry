/** @file esp_wifi.h — host mock. */
#pragma once
#include <cstdint>
#include "esp_err.h"
typedef enum { WIFI_SECOND_CHAN_NONE = 0 } wifi_second_chan_t;
esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
