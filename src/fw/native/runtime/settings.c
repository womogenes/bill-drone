// The settings store: values for every settings.def entry, loaded from NVS at boot, changed by requests and
// calibration, and committed to NVS as one blob while the drone is disarmed.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "settings.h"
#include "py/runtime.h"
#include "py/objstr.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
#define LOCK() portENTER_CRITICAL(&lock)
#define UNLOCK() portEXIT_CRITICAL(&lock)

#define RAD (3.14159265358979323846f / 180.0f)
// NVS blob: "FS1", a reserved byte, then (name length, name, float32) pairs. Names rather than positions let
// entries be added or removed without losing the rest.
#define BLOB 1024

static const char *const names[SETTINGS] = {
#define TUNE(name, member, scale, min, max, value) #name,
#define STATE(name, min, max, value) #name,
#include "settings.def"
#undef TUNE
#undef STATE
};
static const float minimum[SETTINGS] = {
#define TUNE(name, member, scale, min, max, value) min,
#define STATE(name, min, max, value) min,
#include "settings.def"
#undef TUNE
#undef STATE
};
static const float maximum[SETTINGS] = {
#define TUNE(name, member, scale, min, max, value) max,
#define STATE(name, min, max, value) max,
#include "settings.def"
#undef TUNE
#undef STATE
};
static const float defaults[SETTINGS] = {
#define TUNE(name, member, scale, min, max, value) value,
#define STATE(name, min, max, value) value,
#include "settings.def"
#undef TUNE
#undef STATE
};
static const bool tunable[SETTINGS] = {
#define TUNE(name, member, scale, min, max, value) true,
#define STATE(name, min, max, value) false,
#include "settings.def"
#undef TUNE
#undef STATE
};
static const int calibration[] = {
    SETTING_gyro_bias_x, SETTING_gyro_bias_y, SETTING_gyro_bias_z, SETTING_pose_zero_roll, SETTING_pose_zero_pitch,
    SETTING_calibrated, SETTING_calibration_frame,
};

static float values[SETTINGS];
static bool dirty, calibration_unsaved;
static uint32_t version;
static int64_t changed_us;

// No formatting or allocation, so the runtime can call it under its lock.
bool settings_valid(int index, float value) {
    return isfinite(value) && value >= minimum[index] && value <= maximum[index];
}

// NULL if value fits the table entry, otherwise why not; error is scratch space.
const char *settings_validate(int index, float value, char *error, size_t size) {
    if (settings_valid(index, value)) return NULL;
    snprintf(error, size, "%s would be %g, outside %g..%g", names[index], value, minimum[index], maximum[index]);
    return error;
}

// A finished calibration fits the table: gyro bias in rad/s, pose zero in degrees.
bool settings_calibration_fits(const float *bias, const float *pose_zero) {
    return settings_valid(SETTING_gyro_bias_x, bias[0]) && settings_valid(SETTING_gyro_bias_y, bias[1]) &&
           settings_valid(SETTING_gyro_bias_z, bias[2]) && settings_valid(SETTING_pose_zero_roll, pose_zero[0]) &&
           settings_valid(SETTING_pose_zero_pitch, pose_zero[1]);
}

int settings_index(const char *name, size_t length) {
    for (int i = 0; i < SETTINGS; i++) {
        if (strlen(names[i]) == length && !memcmp(names[i], name, length)) return i;
    }
    return -1;
}

// Applies a parsed blob; anything missing, unknown, out of range or truncated keeps its default, and a calibration
// that is not complete and in range is not calibrated.
void settings_decode(const uint8_t *data, size_t size) {
    float loaded[SETTINGS];
    bool present[SETTINGS] = {false};
    memcpy(loaded, defaults, sizeof(loaded));
    if (size >= 4 && !memcmp(data, "FS1", 3)) {
        for (size_t at = 4; at < size;) {
            size_t length = data[at];
            if (at + 1 + length + 4 > size) break;
            int index = settings_index((const char *)data + at + 1, length);
            float value;
            memcpy(&value, data + at + 1 + length, 4);
            if (index >= 0 && settings_valid(index, value)) {
                loaded[index] = value;
                present[index] = true;
            }
            at += 1 + length + 4;
        }
    }
    bool complete = true;
    for (size_t i = 0; i < sizeof(calibration) / sizeof(calibration[0]); i++) complete &= present[calibration[i]];
    if (!complete || loaded[SETTING_calibrated] != 1) {
        for (size_t i = 0; i < sizeof(calibration) / sizeof(calibration[0]); i++) loaded[calibration[i]] = defaults[calibration[i]];
    }
    LOCK();
    memcpy(values, loaded, sizeof(values));
    dirty = calibration_unsaved = false;
    version++;
    UNLOCK();
}

size_t settings_encode(uint8_t *data, size_t size) {
    float copy[SETTINGS];
    LOCK();
    memcpy(copy, values, sizeof(copy));
    UNLOCK();
    memcpy(data, "FS1", 4);
    size_t at = 4;
    for (int i = 0; i < SETTINGS; i++) {
        size_t length = strlen(names[i]);
        if (at + 1 + length + 4 > size) return 0;
        data[at] = length;
        memcpy(data + at + 1, names[i], length);
        memcpy(data + at + 1 + length, &copy[i], 4);
        at += 1 + length + 4;
    }
    return at;
}

void settings_copy(float *output) {
    LOCK();
    memcpy(output, values, sizeof(values));
    UNLOCK();
}

float settings_get(int index) {
    LOCK();
    float value = values[index];
    UNLOCK();
    return value;
}

// Applies all of the values or, if any is outside the table's range, none of them; calibration marks a save the
// dashboard waits for.
bool settings_change(const int *indices, const float *changes, int count, int64_t now) {
    for (int i = 0; i < count; i++) {
        if (!settings_valid(indices[i], changes[i])) return false;
    }
    LOCK();
    for (int i = 0; i < count; i++) {
        values[indices[i]] = changes[i];
        if (indices[i] == SETTING_calibrated) calibration_unsaved = true;
    }
    dirty = true;
    changed_us = now;
    version++;
    UNLOCK();
    return true;
}

uint32_t settings_version(void) {
    LOCK();
    uint32_t value = version;
    UNLOCK();
    return value;
}

bool settings_calibration_unsaved(void) {
    LOCK();
    bool value = calibration_unsaved;
    UNLOCK();
    return value;
}

// The stored calibration, only if it is complete and was taken in sensor_frame; otherwise the IMU is uncalibrated.
bool settings_calibration(float sensor_frame, float *bias, float *pose_zero) {
    float copy[SETTINGS];
    settings_copy(copy);
    if (copy[SETTING_calibrated] != 1 || copy[SETTING_calibration_frame] != sensor_frame) return false;
    bias[0] = copy[SETTING_gyro_bias_x];
    bias[1] = copy[SETTING_gyro_bias_y];
    bias[2] = copy[SETTING_gyro_bias_z];
    pose_zero[0] = copy[SETTING_pose_zero_roll];
    pose_zero[1] = copy[SETTING_pose_zero_pitch];
    return true;
}

void settings_apply(Command *command, const float *source) {
#define TUNE(name, member, scale, min, max, value) command->member = source[SETTING_##name] * (scale);
#define STATE(name, min, max, value)
#include "settings.def"
#undef TUNE
#undef STATE
}

static int number(char *text, size_t size, float value) {
    return snprintf(text, size, "%.9g", value);
}

// {"name":value,...} for every entry, in table order; the S reply and the T reply.
size_t settings_json(char *text, size_t size) {
    float copy[SETTINGS];
    settings_copy(copy);
    size_t at = 0;
    for (int i = 0; i < SETTINGS && at < size; i++) {
        at += snprintf(text + at, size - at, "%s\"%s\":", i ? "," : "{", names[i]);
        if (at < size) at += number(text + at, size - at, copy[i]);
    }
    if (at < size) at += snprintf(text + at, size - at, "}");
    return at < size ? at : 0;
}

// One page of [name, min, max, default, tunable] entries, so every reply fits one request reply.
size_t settings_schema(char *text, size_t size, int page) {
    int first = page * SCHEMA_PAGE;
    size_t at = snprintf(text, size, "{\"total\":%d,\"first\":%d,\"settings\":[", SETTINGS, first);
    for (int i = first; i < SETTINGS && i < first + SCHEMA_PAGE && at < size; i++) {
        at += snprintf(text + at, size - at, "%s[\"%s\",", i > first ? "," : "", names[i]);
        if (at < size) at += number(text + at, size - at, minimum[i]);
        if (at < size) at += snprintf(text + at, size - at, ",");
        if (at < size) at += number(text + at, size - at, maximum[i]);
        if (at < size) at += snprintf(text + at, size - at, ",");
        if (at < size) at += number(text + at, size - at, defaults[i]);
        if (at < size) at += snprintf(text + at, size - at, ",%s]", tunable[i] ? "true" : "false");
    }
    if (at < size) at += snprintf(text + at, size - at, "]}");
    return at < size ? at : 0;
}

static const char *skip(const char *at, const char *end) {
    while (at < end && (*at == ' ' || *at == '\t' || *at == '\n' || *at == '\r')) at++;
    return at;
}

// Parses a flat JSON object of numbers and applies all of it or none of it. Returns an error or NULL.
const char *settings_tune(const char *json, size_t length, int64_t now, char *error, size_t size) {
    int indices[SETTINGS];
    float changes[SETTINGS];
    int count = 0;
    const char *at = skip(json, json + length), *end = json + length;
    if (at == end || *at++ != '{') return "Expected a JSON object";
    at = skip(at, end);
    if (at < end && *at == '}') return "Expected at least one setting";
    while (at < end) {
        if (*at++ != '"') return "Expected a setting name";
        const char *name = at;
        while (at < end && *at != '"' && *at != '\\') at++;
        if (at == end || *at != '"') return "Malformed setting name";
        size_t name_length = at++ - name;
        at = skip(at, end);
        if (at == end || *at++ != ':') return "Expected ':'";
        char digits[32];
        at = skip(at, end);
        size_t used = 0;
        while (at < end && used < sizeof(digits) - 1 && strchr("+-.0123456789eE", *at)) digits[used++] = *at++;
        digits[used] = 0;
        char *parsed;
        float value = strtof(digits, &parsed);
        int index = settings_index(name, name_length);
        if (index < 0) {
            snprintf(error, size, "Unknown setting %.*s", (int)name_length, name);
            return error;
        }
        if (!tunable[index]) {
            snprintf(error, size, "%s is read-only", names[index]);
            return error;
        }
        if (!used || *parsed) {
            snprintf(error, size, "%s needs a number", names[index]);
            return error;
        }
        if (!settings_valid(index, value)) {
            snprintf(error, size, "%s must be between %g and %g", names[index], minimum[index], maximum[index]);
            return error;
        }
        if (count == SETTINGS) return "Too many settings";
        indices[count] = index;
        changes[count++] = value;
        at = skip(at, end);
        if (at < end && *at == ',') {
            at = skip(at + 1, end);
            continue;
        }
        if (at < end && *at == '}' && skip(at + 1, end) == end) {
            settings_change(indices, changes, count, now);
            return NULL;
        }
        return "Expected ',' or '}'";
    }
    return "Unterminated JSON object";
}

static uint8_t blob[BLOB];

void settings_load(void) {
    nvs_handle_t handle;
    size_t size = sizeof(blob);
    esp_err_t error = nvs_open("flight", NVS_READONLY, &handle);
    if (!error) {
        error = nvs_get_blob(handle, "settings", blob, &size);
        nvs_close(handle);
    }
    settings_decode(blob, error ? 0 : size);
    if (error && error != ESP_ERR_NVS_NOT_FOUND) printf("Settings: NVS read failed: %s\n", esp_err_to_name(error));
}

bool settings_save_due(int64_t now) {
    LOCK();
    bool due = dirty && now - changed_us > 1000000;
    UNLOCK();
    return due;
}

// Called by the radio's update task only, between flight_runtime_begin_maintenance and end: the NVS write stalls
// flash access on both cores, so the outputs are held at zero and it never runs under the control lock.
void settings_commit(int64_t now) {
    LOCK();
    dirty = false;
    bool calibration_saving = calibration_unsaved;
    UNLOCK();
    size_t size = settings_encode(blob, sizeof(blob));
    nvs_handle_t handle;
    esp_err_t error = nvs_open("flight", NVS_READWRITE, &handle);
    if (!error) {
        error = nvs_set_blob(handle, "settings", blob, size);
        if (!error) error = nvs_commit(handle);
        nvs_close(handle);
    }
    LOCK();
    if (error) {
        dirty = true;
        changed_us = now;
    } else if (calibration_saving) {
        calibration_unsaved = false;
    }
    UNLOCK();
    if (error) printf("Settings: NVS write failed: %s\n", esp_err_to_name(error));
}

// Python API, for recording metadata.
static mp_obj_t get_python(mp_obj_t name) {
    size_t length;
    const char *text = mp_obj_str_get_data(name, &length);
    int index = settings_index(text, length);
    if (index < 0) mp_raise_ValueError(MP_ERROR_TEXT("Unknown setting"));
    return mp_obj_new_float(settings_get(index));
}
static MP_DEFINE_CONST_FUN_OBJ_1(get_obj, get_python);

static mp_obj_t json_python(void) {
    char text[1536];
    size_t size = settings_json(text, sizeof(text));
    return mp_obj_new_str(text, size);
}
static MP_DEFINE_CONST_FUN_OBJ_0(json_obj, json_python);

static mp_obj_t version_python(void) {
    return mp_obj_new_int_from_uint(settings_version());
}
static MP_DEFINE_CONST_FUN_OBJ_0(version_obj, version_python);

static const mp_rom_map_elem_t globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_settings)},
    {MP_ROM_QSTR(MP_QSTR_get), MP_ROM_PTR(&get_obj)},
    {MP_ROM_QSTR(MP_QSTR_json), MP_ROM_PTR(&json_obj)},
    {MP_ROM_QSTR(MP_QSTR_version), MP_ROM_PTR(&version_obj)},
};
static MP_DEFINE_CONST_DICT(globals, globals_table);
const mp_obj_module_t settings_module = {.base = {&mp_type_module}, .globals = (mp_obj_dict_t *)&globals};
MP_REGISTER_MODULE(MP_QSTR_settings, settings_module);
