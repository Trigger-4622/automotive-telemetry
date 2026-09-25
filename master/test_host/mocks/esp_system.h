/**
 * @file esp_system.h
 * @brief Host mock of the ESP-IDF reset-reason API. The simulated value is set
 *        by the scenarios (sim::resetReason) so a brownout during cranking can
 *        be replayed on the PC.
 */
#pragma once

typedef enum {
    ESP_RST_UNKNOWN = 0,
    ESP_RST_POWERON,
    ESP_RST_EXT,
    ESP_RST_SW,
    ESP_RST_PANIC,
    ESP_RST_INT_WDT,
    ESP_RST_TASK_WDT,
    ESP_RST_WDT,
    ESP_RST_DEEPSLEEP,
    ESP_RST_BROWNOUT,
    ESP_RST_SDIO,
} esp_reset_reason_t;

esp_reset_reason_t esp_reset_reason(void);
