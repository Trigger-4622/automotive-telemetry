/** @file semphr.h — host mock. Misuse (giving a mutex you do not hold) is
 *  reported as a firmware bug. */
#pragma once
#include "freertos/FreeRTOS.h"

typedef struct SimSem *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex();
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex();
SemaphoreHandle_t xSemaphoreCreateBinary();
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t s, TickType_t ticks);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t s);
void vSemaphoreDelete(SemaphoreHandle_t s);
