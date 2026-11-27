/**
 * @file esp_partition.h
 * @brief ESP-IDF's raw partition access for the host tests: the "logs"
 *        partition as bytes (simfs::logsRaw), with NOR-flash rules - a write
 *        can only clear bits (one that would set any is counted in
 *        simfs::flashWriteViolations), erases are whole 4 KB sectors - and
 *        tied to the LittleFS mock: erasing its superblocks leaves LittleFS
 *        nothing to mount (simfs::logsBlank), as on the chip.
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_err.h"

typedef enum { ESP_PARTITION_TYPE_APP = 0x00, ESP_PARTITION_TYPE_DATA = 0x01 } esp_partition_type_t;
typedef enum { ESP_PARTITION_SUBTYPE_DATA_SPIFFS = 0x82, ESP_PARTITION_SUBTYPE_ANY = 0xff } esp_partition_subtype_t;
typedef struct {
    char     label[17];
    uint32_t address;
    uint32_t size;
} esp_partition_t;

const esp_partition_t *esp_partition_find_first(esp_partition_type_t type, esp_partition_subtype_t subtype,
                                                const char *label);
esp_err_t esp_partition_read(const esp_partition_t *p, size_t offset, void *dst, size_t size);
esp_err_t esp_partition_write(const esp_partition_t *p, size_t offset, const void *src, size_t size);
esp_err_t esp_partition_erase_range(const esp_partition_t *p, size_t offset, size_t size);
