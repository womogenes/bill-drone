#pragma once

#include <stddef.h>
#include "esp_err.h"

// No flash on the host: every open fails as an empty store does, so the settings start from their defaults.
typedef int nvs_handle_t;
#define NVS_READONLY 0
#define NVS_READWRITE 1
static inline esp_err_t nvs_open(const char *name, int mode, nvs_handle_t *handle) { (void)name, (void)mode, (void)handle; return ESP_ERR_NVS_NOT_FOUND; }
static inline esp_err_t nvs_get_blob(nvs_handle_t handle, const char *key, void *data, size_t *size) { (void)handle, (void)key, (void)data, (void)size; return ESP_ERR_NVS_NOT_FOUND; }
static inline esp_err_t nvs_set_blob(nvs_handle_t handle, const char *key, const void *data, size_t size) { (void)handle, (void)key, (void)data, (void)size; return ESP_FAIL; }
static inline esp_err_t nvs_commit(nvs_handle_t handle) { (void)handle; return ESP_FAIL; }
static inline void nvs_close(nvs_handle_t handle) { (void)handle; }
