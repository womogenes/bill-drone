// Host stand-ins for the parts of ESP-IDF the control task uses (drone/test/flight_rt_test.c): no threads, so a lock only counts how deep it is held, which the test checks is zero between operations.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef struct { int depth; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define configTICK_RATE_HZ 100
#define pdMS_TO_TICKS(ms) ((TickType_t)((ms) * configTICK_RATE_HZ / 1000))
#define pdPASS 1
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY 0xffffffffu

extern int host_lock_depth;
#define portENTER_CRITICAL(mux) ((mux)->depth++, host_lock_depth++)
#define portEXIT_CRITICAL(mux) ((mux)->depth--, host_lock_depth--)
#define portENTER_CRITICAL_ISR(mux) portENTER_CRITICAL(mux)
#define portEXIT_CRITICAL_ISR(mux) portEXIT_CRITICAL(mux)
