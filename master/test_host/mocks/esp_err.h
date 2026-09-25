/** @file esp_err.h — host mock (values as in ESP-IDF). */
#pragma once
typedef int esp_err_t;
#define ESP_OK                 0
#define ESP_FAIL              -1
#define ESP_ERR_NO_MEM         0x101
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_NOT_SUPPORTED  0x106
#define ESP_ERR_TIMEOUT        0x107

typedef enum { GPIO_NUM_NC = -1, GPIO_NUM_0 = 0, GPIO_NUM_MAX = 49 } gpio_num_t;
