#pragma once
/**
 * @file esp_rom_gpio.h
 * @brief Mock of ESP-IDF's ROM GPIO-matrix routing - the one call the master
 *        makes to give the CAN TX pad back to the controller (sim_bus.cpp).
 */
#include <cstdint>

void esp_rom_gpio_connect_out_signal(uint32_t gpio_num, uint32_t signal_idx, bool out_inv, bool oen_inv);
