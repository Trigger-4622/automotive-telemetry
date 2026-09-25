/**
 * @file twai.h
 * @brief Host mock of the ESP-IDF 4.4 TWAI driver API: same types, constants
 *        and functions (copied from driver/twai.h and hal/twai_types.h), backed
 *        by the simulated CAN bus in sim_bus.cpp.
 */
#pragma once
#include <cstdint>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define TWAI_FRAME_MAX_DLC 8
#define TWAI_IO_UNUSED ((gpio_num_t)-1)
#define ESP_INTR_FLAG_LEVEL1 (1 << 1)

#define TWAI_ALERT_TX_IDLE               0x00000001
#define TWAI_ALERT_TX_SUCCESS            0x00000002
#define TWAI_ALERT_RX_DATA               0x00000004
#define TWAI_ALERT_BELOW_ERR_WARN        0x00000008
#define TWAI_ALERT_ERR_ACTIVE            0x00000010
#define TWAI_ALERT_RECOVERY_IN_PROGRESS  0x00000020
#define TWAI_ALERT_BUS_RECOVERED         0x00000040
#define TWAI_ALERT_ARB_LOST              0x00000080
#define TWAI_ALERT_ABOVE_ERR_WARN        0x00000100
#define TWAI_ALERT_BUS_ERROR             0x00000200
#define TWAI_ALERT_TX_FAILED             0x00000400
#define TWAI_ALERT_RX_QUEUE_FULL         0x00000800
#define TWAI_ALERT_ERR_PASS              0x00001000
#define TWAI_ALERT_BUS_OFF               0x00002000
#define TWAI_ALERT_RX_FIFO_OVERRUN       0x00004000
#define TWAI_ALERT_ALL                   0x0001FFFF
#define TWAI_ALERT_NONE                  0x00000000

typedef enum { TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK, TWAI_MODE_LISTEN_ONLY } twai_mode_t;
typedef enum { TWAI_STATE_STOPPED, TWAI_STATE_RUNNING, TWAI_STATE_BUS_OFF,
               TWAI_STATE_RECOVERING } twai_state_t;

typedef struct {
    union {
        struct {
            uint32_t extd : 1;
            uint32_t rtr : 1;
            uint32_t ss : 1;
            uint32_t self : 1;
            uint32_t dlc_non_comp : 1;
            uint32_t reserved : 27;
        };
        uint32_t flags;
    };
    uint32_t identifier;
    uint8_t  data_length_code;
    uint8_t  data[TWAI_FRAME_MAX_DLC];
} twai_message_t;

typedef struct {
    uint32_t brp;
    uint8_t  tseg_1;
    uint8_t  tseg_2;
    uint8_t  sjw;
    bool     triple_sampling;
} twai_timing_config_t;

typedef struct {
    uint32_t acceptance_code;
    uint32_t acceptance_mask;
    bool     single_filter;
} twai_filter_config_t;

typedef struct {
    twai_mode_t mode;
    gpio_num_t  tx_io;
    gpio_num_t  rx_io;
    gpio_num_t  clkout_io;
    gpio_num_t  bus_off_io;
    uint32_t    tx_queue_len;
    uint32_t    rx_queue_len;
    uint32_t    alerts_enabled;
    uint32_t    clkout_divider;
    int         intr_flags;
} twai_general_config_t;

typedef struct {
    twai_state_t state;
    uint32_t msgs_to_tx;
    uint32_t msgs_to_rx;
    uint32_t tx_error_counter;
    uint32_t rx_error_counter;
    uint32_t tx_failed_count;
    uint32_t rx_missed_count;
    uint32_t rx_overrun_count;
    uint32_t arb_lost_count;
    uint32_t bus_error_count;
} twai_status_info_t;

#define TWAI_GENERAL_CONFIG_DEFAULT(tx_io_num, rx_io_num, op_mode) {.mode = op_mode, .tx_io = tx_io_num, .rx_io = rx_io_num, \
    .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 5, .rx_queue_len = 5, \
    .alerts_enabled = TWAI_ALERT_NONE, .clkout_divider = 0, .intr_flags = ESP_INTR_FLAG_LEVEL1}
#define TWAI_TIMING_CONFIG_125KBITS()  {.brp = 32, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_250KBITS()  {.brp = 16, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_500KBITS()  {.brp = 8,  .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_TIMING_CONFIG_1MBITS()    {.brp = 4,  .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true}

esp_err_t twai_driver_install(const twai_general_config_t *g, const twai_timing_config_t *t,
                              const twai_filter_config_t *f);
esp_err_t twai_driver_uninstall(void);
esp_err_t twai_start(void);
esp_err_t twai_stop(void);
esp_err_t twai_transmit(const twai_message_t *message, TickType_t ticks_to_wait);
esp_err_t twai_receive(twai_message_t *message, TickType_t ticks_to_wait);
esp_err_t twai_read_alerts(uint32_t *alerts, TickType_t ticks_to_wait);
esp_err_t twai_initiate_recovery(void);
esp_err_t twai_get_status_info(twai_status_info_t *status_info);
