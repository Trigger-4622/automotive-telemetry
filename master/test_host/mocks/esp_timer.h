/** @file esp_timer.h  @brief esp_timer_get_time() on the simulated clock. */
#pragma once
#include <cstdint>
#include "sim_rtos.h"
inline int64_t esp_timer_get_time() { return (int64_t)simrtos::nowUs(); }
