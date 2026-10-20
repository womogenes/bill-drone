#include <math.h>
#include <stddef.h>

#include "flight_math.h"

static float clamp(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

// Calibration rows are (throttle, thrust). Reverse lookup uses the rising curve.
static float interpolate(float value, const float *table, size_t count, int reverse) {
    if (value <= table[reverse]) return table[1 - reverse];
    for (size_t i = 1; i < count; i++) {
        float x = table[2 * i + reverse];
        if (reverse && x < table[2 * (i - 1) + reverse]) {
            return table[2 * (i - 1)];
        }
        if (value <= x) {
            float x0 = table[2 * (i - 1) + reverse];
            float y0 = table[2 * (i - 1) + 1 - reverse];
            float y1 = table[2 * i + 1 - reverse];
            return y0 + (value - x0) / (x - x0) * (y1 - y0);
        }
    }
    return table[2 * (count - 1) + 1 - reverse];
}

void flight_attitude(Attitude *state, const Motion *m) {
    float roll = state->roll;
    float pitch = state->pitch;
    float sr = sinf(roll), cr = cosf(roll);
    float roll_rate = m->gx + tanf(pitch) * (m->gy * sr + m->gz * cr);
    float pitch_rate = m->gy * cr - m->gz * sr;
    float gravity_roll = atan2f(m->ay, m->az);
    float gravity_pitch = atan2f(-m->ax, sqrtf(m->ay * m->ay + m->az * m->az));
    float weight = 1.0f - expf(-m->dt / m->gravity_tau);

    roll += roll_rate * m->dt;
    pitch += pitch_rate * m->dt;
    state->roll = roll + weight * (gravity_roll - roll);
    state->pitch = pitch + weight * (gravity_pitch - pitch);
}

#define MIN_THRUST 0.3f

// Integrators start from the configured bias so liftoff already counters a known disturbance.
void flight_reset(Controller *state, const Command *c) {
    state->roll_i = clamp(c->roll_bias, -c->integral_limit, c->integral_limit);
    state->pitch_i = clamp(c->pitch_bias, -c->integral_limit, c->integral_limit);
}

void flight_control(Controller *state, const Command *c, const float *table, size_t count) {
    if (!c->enabled || c->base == 0) {
        flight_reset(state, c);
        state->left = state->right = state->left_angle = state->right_angle = 0;
        return;
    }

    const Model *model = &c->model;
    float trim = c->pitch_trim;
    float force = 2 * interpolate(c->base, table, count, 0) * cosf(trim);
    // Without thrust, atan2(horizontal, ~0) saturates the servos on gyro noise.
    if (force < 2 * MIN_THRUST) {
        flight_reset(state, c);
        state->left = state->right = c->base;
        state->left_angle = clamp(c->left_trim, model->servo_min, model->servo_max);
        state->right_angle = clamp(c->right_trim, model->servo_min, model->servo_max);
        return;
    }
    float roll_error = c->target_roll - c->roll;
    float pitch_error = c->target_pitch + trim - c->pitch;
    state->roll_i = clamp(state->roll_i + c->roll_i * roll_error * c->dt,
                          -c->integral_limit, c->integral_limit);
    state->pitch_i = clamp(state->pitch_i + c->pitch_i * pitch_error * c->dt,
                           -c->integral_limit, c->integral_limit);

    float roll_torque = model->roll_inertia * (c->roll_p * roll_error + state->roll_i - c->roll_damping * c->gx);
    float pitch_torque = model->pitch_inertia * (c->pitch_p * pitch_error + state->pitch_i - c->pitch_damping * c->gy);
    float yaw_torque = model->yaw_inertia * (c->yaw_damping * (c->target_yaw - c->gz) + c->yaw_bias);

    float left_vertical = fmaxf(0, (force + roll_torque / model->arm) / 2);
    float right_vertical = fmaxf(0, (force - roll_torque / model->arm) / 2);
    float forward = (pitch_torque - model->com_x * force) / model->pivot_height;
    yaw_torque -= model->reaction_torque_ratio * (right_vertical - left_vertical);
    float left_horizontal = (forward - yaw_torque / model->arm) / 2;
    float right_horizontal = (forward + yaw_torque / model->arm) / 2;

    state->left_angle = clamp(model->left_servo_direction * atan2f(left_horizontal, left_vertical) + c->left_trim,
                              model->servo_min, model->servo_max);
    state->right_angle = clamp(model->right_servo_direction * atan2f(right_horizontal, right_vertical) + c->right_trim,
                               model->servo_min, model->servo_max);
    state->left = roundf(interpolate(hypotf(left_horizontal, left_vertical) / c->left_thrust_scale, table, count, 1));
    state->right = roundf(interpolate(hypotf(right_horizontal, right_vertical) / c->right_thrust_scale, table, count, 1));
}

#ifdef MICROPY_ENABLE_DYNRUNTIME
#include "py/dynruntime.h"

// libm's errno ABI; this module has no dependency on the C library's thread state.
static int math_error;
int *math_errno(void) __asm__("__errno");
int *math_errno(void) { return &math_error; }

static void *buffer(mp_obj_t object, size_t size, int flags) {
    mp_buffer_info_t data;
    mp_get_buffer_raise(object, &data, flags);
    if (data.len != size) mp_raise_ValueError(MP_ERROR_TEXT("Invalid flight buffer size"));
    return data.buf;
}

static mp_obj_t *vector(mp_obj_t object, size_t expected) {
    size_t count;
    mp_obj_t *items;
    mp_obj_get_array(object, &count, &items);
    if (count != expected) mp_raise_ValueError(MP_ERROR_TEXT("Invalid vector size"));
    return items;
}

static mp_obj_t attitude_step(size_t count, const mp_obj_t *args) {
    Attitude *state = buffer(args[0], sizeof(Attitude), MP_BUFFER_WRITE);
    mp_obj_t *accel = vector(args[1], 3);
    mp_obj_t *gyro = vector(args[2], 3);
    Motion motion = {
        mp_obj_get_float(accel[0]), mp_obj_get_float(accel[1]), mp_obj_get_float(accel[2]),
        mp_obj_get_float(gyro[0]), mp_obj_get_float(gyro[1]), mp_obj_get_float(gyro[2]),
        mp_obj_get_float(args[3]), mp_obj_get_float(args[4]),
    };
    flight_attitude(state, &motion);
    mp_obj_t result[] = {
        mp_obj_new_float(state->roll), mp_obj_new_float(state->pitch),
    };
    return mp_obj_new_tuple(2, result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(attitude_step_obj, 5, 5, attitude_step);

static mp_obj_t control_step(size_t count, const mp_obj_t *args) {
    Controller *state = buffer(args[0], sizeof(Controller), MP_BUFFER_WRITE);
    Command *command = buffer(args[1], sizeof(Command), MP_BUFFER_WRITE);
    mp_buffer_info_t table;
    mp_get_buffer_raise(args[2], &table, MP_BUFFER_READ);
    if (table.len < 16 || table.len % 8) mp_raise_ValueError(MP_ERROR_TEXT("Invalid thrust table"));

    mp_obj_t *attitude = vector(args[3], 2);
    mp_obj_t *gyro = vector(args[4], 3);
    command->roll = mp_obj_get_float(attitude[0]);
    command->pitch = mp_obj_get_float(attitude[1]);
    command->gx = mp_obj_get_float(gyro[0]);
    command->gy = mp_obj_get_float(gyro[1]);
    command->gz = mp_obj_get_float(gyro[2]);
    command->dt = mp_obj_get_float(args[5]);
    flight_control(state, command, table.buf, table.len / 8);
    mp_obj_t result[] = {
        mp_obj_new_int((int)state->left), mp_obj_new_int((int)state->right),
        mp_obj_new_float(state->left_angle), mp_obj_new_float(state->right_angle),
    };
    return mp_obj_new_tuple(4, result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(control_step_obj, 6, 6, control_step);

mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args) {
    MP_DYNRUNTIME_INIT_ENTRY
    mp_store_global(MP_QSTR_attitude_step, MP_OBJ_FROM_PTR(&attitude_step_obj));
    mp_store_global(MP_QSTR_control_step, MP_OBJ_FROM_PTR(&control_step_obj));
    MP_DYNRUNTIME_INIT_EXIT
}
#endif
