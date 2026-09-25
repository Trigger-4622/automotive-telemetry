/** @file esp_now.h — host mock; sent packets go to the simulated displays. */
#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_err.h"

typedef enum { WIFI_IF_STA = 0, WIFI_IF_AP = 1 } wifi_interface_t;
typedef struct {
    uint8_t peer_addr[6];
    uint8_t lmk[16];
    uint8_t channel;
    wifi_interface_t ifidx;
    bool encrypt;
    void *priv;
} esp_now_peer_info_t;

esp_err_t esp_now_init(void);
esp_err_t esp_now_deinit(void);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *peer);
esp_err_t esp_now_send(const uint8_t *peer_addr, const uint8_t *data, size_t len);
