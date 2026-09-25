/**
 * @file gpio.h
 * @brief Host mock of the ESP-IDF GPIO hold API. Only what the master needs to
 *        keep the CAN TX line recessive across reset and deep sleep. The mock
 *        records what was asked of it (sim::gpioHoldEnabled, sim::gpioHoldPin)
 *        so the scenarios can check the pin is latched high while asleep.
 */
#pragma once
#include "esp_err.h"

esp_err_t gpio_hold_en(gpio_num_t pin);
esp_err_t gpio_hold_dis(gpio_num_t pin);
void      gpio_deep_sleep_hold_en(void);
void      gpio_deep_sleep_hold_dis(void);
esp_err_t gpio_pullup_en(gpio_num_t pin);
