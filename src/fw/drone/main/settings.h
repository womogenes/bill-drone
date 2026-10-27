#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "flight_math.h"

enum {
#define TUNE(name, member, scale, min, max, value) SETTING_##name,
#define STATE(name, min, max, value) SETTING_##name,
#include "settings.def"
#undef TUNE
#undef STATE
    SETTINGS
};

// The longest settings_json text, so its buffers grow with the list: per entry a separator, two quotes and a colon (4), the name, and a %.9g number (at most 15, as -1.17549435e-38), then the closing brace and the terminator.
enum {
    SETTINGS_NAMES = sizeof(""
#define TUNE(name, ...) #name
#define STATE(name, ...) #name
#include "settings.def"
#undef TUNE
#undef STATE
                     ) - 1
};
#define SETTINGS_JSON_SIZE (SETTINGS_NAMES + 19 * SETTINGS + 2)

// Schema entries per D reply page: about 60 bytes each, well inside the request reply.
#define SCHEMA_PAGE 12

int settings_index(const char *name, size_t length);
void settings_decode(const uint8_t *data, size_t size);
size_t settings_encode(uint8_t *data, size_t size);
void settings_copy(float *values);
float settings_get(int index);
// False, changing nothing, if any value is outside its table range.
bool settings_change(const int *indices, const float *values, int count, int64_t now);
bool settings_valid(int index, float value);
// NULL if value fits entry index, otherwise a message in error.
const char *settings_validate(int index, float value, char *error, size_t size);
bool settings_calibration_fits(const float *bias, const float *pose_zero);
uint32_t settings_version(void);
bool settings_calibration_unsaved(void);
// The stored calibration: false unless it is complete and was taken in sensor_frame.
bool settings_calibration(float sensor_frame, float *bias, float *pose_zero);
// Copies the TUNE entries into the controller's command, converting units.
void settings_apply(Command *command, const float *values);
// Both return 0 rather than a truncated reply when the text does not fit.
size_t settings_json(char *text, size_t size);
size_t settings_schema(char *text, size_t size, int page);
// error is scratch space for the returned message.
const char *settings_tune(const char *json, size_t length, int64_t now, char *error, size_t size);
void settings_load(void);
bool settings_save_due(int64_t now);
// Only between flight_runtime_begin_maintenance and flight_runtime_end_maintenance.
void settings_commit(int64_t now);

#endif
