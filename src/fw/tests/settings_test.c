// Host test of the native settings store: cc -DSETTINGS_HOST tests/settings_test.c -lm (make -C src/fw test).
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../native/runtime/settings.c"

// radio.c REPLY minus the request id and status byte.
#define REPLY_TEXT (8 * 232 - 5)

static float value(const char *name) {
    return settings_get(settings_index(name, strlen(name)));
}

static const char *tune(const char *json) {
    static char error[96];
    return settings_tune(json, strlen(json), 0, error, sizeof(error));
}

static void set(const char *name, float number) {
    int index = settings_index(name, strlen(name));
    settings_change(&index, &number, 1, 0);
}

// A stored calibration taken in frame 6, the drone's sensor_to_body (1, -1, -1).
static size_t calibrated_blob(uint8_t *blob, size_t size) {
    settings_decode(NULL, 0);
    set("gyro_bias_x", 0.057199492f);
    set("gyro_bias_y", -0.026181264f);
    set("gyro_bias_z", -0.07149655f);
    set("pose_zero_roll", 0.233961216f);
    set("pose_zero_pitch", -7.17778836f);
    set("calibration_frame", 6);
    set("calibrated", 1);
    return settings_encode(blob, size);
}

// Replaces one entry's value inside an encoded blob, as a corrupt or out-of-range NVS entry would.
static void poke(uint8_t *blob, size_t size, const char *name, float number) {
    for (size_t at = 4; at < size; at += 1 + blob[at] + 4) {
        if (blob[at] == strlen(name) && !memcmp(blob + at + 1, name, blob[at])) {
            memcpy(blob + at + 1 + blob[at], &number, 4);
            return;
        }
    }
    assert(!"entry present");
}

static void drop(uint8_t *blob, size_t *size, const char *name) {
    for (size_t at = 4; at < *size; at += 1 + blob[at] + 4) {
        size_t length = 1 + blob[at] + 4;
        if (blob[at] == strlen(name) && !memcmp(blob + at + 1, name, blob[at])) {
            memmove(blob + at, blob + at + length, *size - at - length);
            *size -= length;
            return;
        }
    }
    assert(!"entry present");
}

static bool calibration_valid(float sensor_frame) {
    float bias[3], pose_zero[2];
    return settings_calibration(sensor_frame, bias, pose_zero);
}

int main(void) {
    uint8_t blob[BLOB];
    char text[4096];

    // An empty NVS: defaults, not imported, not calibrated.
    settings_decode(NULL, 0);
    assert(value("roll_p") == 20 && value("servo_offset_left") == -3 && !migrated && !calibration_valid(6));

    // Round trip, including the import mark.
    set("roll_i", 50);
    migrated = true;
    size_t size = settings_encode(blob, sizeof(blob));
    settings_decode(NULL, 0);
    settings_decode(blob, size);
    assert(value("roll_i") == 50 && migrated);

    // A corrupt blob falls back to defaults and asks for the import again.
    blob[0] = 'X';
    settings_decode(blob, size);
    assert(value("roll_i") == 3 && !migrated);

    // Tuning: all or nothing, with clear errors.
    settings_decode(NULL, 0);
    assert(!tune("{\"integral_limit\": 8.5, \"roll_p\":40}") && value("integral_limit") == 8.5f && value("roll_p") == 40);
    assert(!strcmp(tune("{\"roll_p\": 41, \"bogus\": 1}"), "Unknown setting bogus") && value("roll_p") == 40);
    assert(!strcmp(tune("{\"gyro_bias_x\": 0.1}"), "gyro_bias_x is read-only"));
    assert(!strcmp(tune("{\"com_x\": 0.5}"), "com_x must be between -0.02 and 0.02"));
    assert(!strcmp(tune("{\"roll_p\": \"x\"}"), "roll_p needs a number"));
    assert(tune("{\"roll_p\": 1") && tune("[]") && tune("{}") && value("roll_p") == 40);

    // Calibration is valid only complete, in range and in the frame it was taken in.
    size = calibrated_blob(blob, sizeof(blob));
    settings_decode(blob, size);
    assert(calibration_valid(6) && value("pose_zero_pitch") == -7.17778836f);
    assert(!calibration_valid(0));
    poke(blob, size, "gyro_bias_y", 5);
    settings_decode(blob, size);
    assert(!calibration_valid(6) && value("calibrated") == 0 && value("gyro_bias_x") == 0);
    size = calibrated_blob(blob, sizeof(blob));
    poke(blob, size, "pose_zero_roll", NAN);
    settings_decode(blob, size);
    assert(!calibration_valid(6));
    size = calibrated_blob(blob, sizeof(blob));
    drop(blob, &size, "calibration_frame");
    settings_decode(blob, size);
    assert(!calibration_valid(6));
    size = calibrated_blob(blob, sizeof(blob));
    poke(blob, size, "calibrated", 0);
    settings_decode(blob, size);
    assert(!calibration_valid(6));
    size = calibrated_blob(blob, sizeof(blob)) - 3;
    settings_decode(blob, size);
    assert(!calibration_valid(6));

    // Servo zeroing and calibration results are checked against the table before anything changes.
    settings_decode(NULL, 0);
    set("servo_offset_left", 8);
    char message[96];
    int left = SETTING_servo_offset_left;
    assert(!strcmp(settings_validate(left, 8 + 60, message, sizeof(message)), "servo_offset_left would be 68, outside -60..60"));
    float invalid = 68;
    assert(!settings_change(&left, &invalid, 1, 0) && value("servo_offset_left") == 8);
    assert(!settings_validate(left, 8 + 50, message, sizeof(message)));
    float valid_offset = 58;
    assert(settings_change(&left, &valid_offset, 1, 0) && value("servo_offset_left") == 58);
    assert(!settings_validate(left, 60, message, sizeof(message)) && !settings_validate(left, -60, message, sizeof(message)));
    assert(settings_validate(left, 60.0001f, message, sizeof(message)) && settings_validate(left, NAN, message, sizeof(message)));
    int both[] = {SETTING_roll_p, SETTING_servo_offset_right};
    float mixed[] = {30, NAN};
    assert(!settings_change(both, mixed, 2, 0) && value("roll_p") == 20 && value("servo_offset_right") == 0);
    float bias[] = {0.05f, -0.02f, -0.07f}, pose_zero[] = {0.2f, -7.2f};
    assert(settings_calibration_fits(bias, pose_zero));
    bias[1] = 1.5f;
    assert(!settings_calibration_fits(bias, pose_zero));
    bias[1] = -0.02f;
    pose_zero[1] = NAN;
    assert(!settings_calibration_fits(bias, pose_zero));
    pose_zero[1] = 95;
    assert(!settings_calibration_fits(bias, pose_zero));

    // Replies always fit, with the longest values: every page holds SCHEMA_PAGE entries and names never repeat.
    // Worst case regardless of range, written straight into the store.
    for (int i = 0; i < SETTINGS; i++) values[i] = -0.0000123456789f;
    assert(settings_json(text, REPLY_TEXT) && settings_json(text, 1536));
    int seen = 0;
    for (int page = 0; page * SCHEMA_PAGE < SETTINGS; page++) {
        size_t length = settings_schema(text, REPLY_TEXT, page);
        assert(length);
        for (int i = page * SCHEMA_PAGE; i < SETTINGS && i < (page + 1) * SCHEMA_PAGE; i++, seen++) {
            char quoted[64];
            snprintf(quoted, sizeof(quoted), "[\"%s\",", names[i]);
            assert(strstr(text, quoted));
        }
    }
    assert(seen == SETTINGS);
    printf("settings: %d entries, %zu-byte settings reply, schema pages fit %d bytes\n", SETTINGS,
           settings_json(text, sizeof(text)), REPLY_TEXT);
    return 0;
}
