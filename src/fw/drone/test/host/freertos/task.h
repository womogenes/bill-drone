#pragma once

#include "freertos/FreeRTOS.h"

TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *previous, TickType_t period);
void vTaskDelay(TickType_t ticks);
BaseType_t xTaskCreatePinnedToCore(void (*task)(void *), const char *name, uint32_t stack, void *arg, int priority, TaskHandle_t *handle, int core);
