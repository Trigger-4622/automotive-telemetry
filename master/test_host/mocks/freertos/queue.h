/** @file queue.h — host mock. */
#pragma once
#include "freertos/FreeRTOS.h"

typedef struct SimQueue *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t itemSize);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks);
BaseType_t xQueueReceive(QueueHandle_t q, void *buf, TickType_t ticks);
void vQueueDelete(QueueHandle_t q);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q);
