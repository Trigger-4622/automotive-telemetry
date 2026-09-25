/** @file esp_sleep.h — host mock; deep sleep ends the simulation run. */
#pragma once
#include "esp_err.h"
typedef enum {
    ESP_SLEEP_WAKEUP_UNDEFINED, ESP_SLEEP_WAKEUP_ALL, ESP_SLEEP_WAKEUP_EXT0,
    ESP_SLEEP_WAKEUP_EXT1, ESP_SLEEP_WAKEUP_TIMER
} esp_sleep_wakeup_cause_t;
esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause(void);
esp_err_t esp_sleep_enable_ext0_wakeup(gpio_num_t pin, int level);
[[noreturn]] void esp_deep_sleep_start(void);
