#pragma once

#include "freertos/FreeRTOS.h"

// A mutex only counts how deep it is held (host_lock_depth), as the critical sections do.
typedef struct { int depth; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *buffer) {
    buffer->depth = 0;
    return buffer;
}
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t wait) {
    (void)wait;
    if (mutex->depth) {
        fprintf(stderr, "mutex taken twice\n");
        abort();
    }
    mutex->depth++;
    host_lock_depth++;
    return pdTRUE;
}
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex) {
    mutex->depth--;
    host_lock_depth--;
    return pdTRUE;
}
