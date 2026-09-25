/** @file FreeRTOS.h — host mock over sim_rtos (see sim_rtos.h). */
#pragma once
#include <cstddef>
#include <cstdint>
#include "sim_rtos.h"

typedef uint32_t TickType_t;
typedef int      BaseType_t;
typedef unsigned UBaseType_t;
typedef void    *TaskHandle_t;

#define pdTRUE   1
#define pdFALSE  0
#define pdPASS   1
#define pdFAIL   0
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFu)
#define configTICK_RATE_HZ 1000
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portTICK_PERIOD_MS 1

/** Ticks to microseconds; portMAX_DELAY means forever. */
inline uint64_t simTicksUs(TickType_t t) {
    return t == portMAX_DELAY ? simrtos::FOREVER : (uint64_t)t * 1000u;
}

typedef struct { int unused; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(m) (void)(m), ++simrtos::g_criticalDepth
#define portEXIT_CRITICAL(m)  (void)(m), --simrtos::g_criticalDepth

typedef void (*TaskFunction_t)(void *);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack,
                                   void *arg, UBaseType_t prio, TaskHandle_t *handle,
                                   BaseType_t core);
void vTaskDelay(TickType_t ticks);
TickType_t xTaskGetTickCount();
