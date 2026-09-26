/**
 * @file twai_ll.h
 * @brief Host mock of the ESP-IDF TWAI low-level register API: just the calls
 *        the master's listen-only erratum fix makes. Implemented against the
 *        simulated controller in sim_bus.cpp (reset mode, REC).
 */
#pragma once
#include <cstdint>

struct twai_dev_t { int unused; };
extern twai_dev_t TWAI;

void     twai_ll_enter_reset_mode(twai_dev_t *hw);
void     twai_ll_exit_reset_mode(twai_dev_t *hw);
bool     twai_ll_is_in_reset_mode(twai_dev_t *hw);
uint32_t twai_ll_get_rec(twai_dev_t *hw);
void     twai_ll_set_rec(twai_dev_t *hw, uint32_t rec);
