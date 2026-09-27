/**
 * @file gpio.h
 * @brief Host mock of the ESP-IDF GPIO API. Only what the master needs to keep
 *        the CAN TX line recessive across reset, sleep and listen-only mode. The mock
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

typedef enum { GPIO_MODE_DISABLE = 0, GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2,
               GPIO_MODE_INPUT_OUTPUT = 3 } gpio_mode_t;
/** Sets the GPIO output latch (what the pad shows once it is a GPIO output). */
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
/** GPIO_MODE_OUTPUT routes the pad to the GPIO latch - away from any peripheral. */
esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode);
/** The pad's level: GPIO0 is the BOOT button (sim::bootButtonDown), any
 *  other pin the TX line. */
int gpio_get_level(gpio_num_t pin);
