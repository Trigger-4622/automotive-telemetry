/**
 * @file twai_ll.h
 * @brief Host mock of the ESP-IDF TWAI low-level register API: just the calls
 *        the master makes - reset mode and the TEC/REC writes, and abort
 *        transmission. Implemented against the simulated controller in
 *        sim_bus.cpp.
 */
#pragma once
#include <cstdint>

struct twai_dev_t {
    /** The error-code-capture register: the latest bus error's segment,
     *  direction (1 = receiving) and type (0 bit, 1 form, 2 stuff, 3 other). */
    union {
        struct { uint32_t seg : 5, dir : 1, errc : 2, reserved8 : 24; };
        uint32_t val;
    } error_code_capture_reg;
};
extern twai_dev_t TWAI;

void     twai_ll_enter_reset_mode(twai_dev_t *hw);
void     twai_ll_exit_reset_mode(twai_dev_t *hw);
bool     twai_ll_is_in_reset_mode(twai_dev_t *hw);
uint32_t twai_ll_get_rec(twai_dev_t *hw);
void     twai_ll_set_rec(twai_dev_t *hw, uint32_t rec);
uint32_t twai_ll_get_tec(twai_dev_t *hw);
void     twai_ll_set_tec(twai_dev_t *hw, uint32_t tec);
/** Abort transmission: the frame in the buffer is not retransmitted after its
 *  current attempt (an aborted frame completes as failed). */
void     twai_ll_set_cmd_abort_tx(twai_dev_t *hw);
