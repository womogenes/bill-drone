// The per-tick physics of sw/sim/rigid.py Sim.step for every env in one call: the identified airframe's actuators and angular accelerations (a port of sw/sim/airframe.py step at sample history - 1, voltage 1 and no floor proximity), then SUB substeps of rigid-body rotation, ground contact and translation. Arrays are contiguous float64 [env, ...]; the operations follow the numpy code's order so both agree to rounding.
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DT 0.01
#define SUB 4
#define G 9.81
#define DEG (3.14159265358979323846 / 180)
#define POINTS 12
// Below this slip speed (m/s) friction is viscous, so the contact stays stable at the substep (rigid.py SLIP).
#define SLIP 0.15

// The scalar parameters, in the order of sw/sim/rigid.py C_PARAMS.
typedef struct {
    double motor_delay, right_motor_delay, motor_tau, right_motor_tau, right_thrust_gain;
    double servo_delay, right_servo_delay, servo_wn, right_servo_wn, servo_zeta, servo_slew;
    double link_wn, link_zeta, inertia_ratio, left_play, right_play, left_band, right_band, band_stiffness, right_gain;
    double roll_gyro, roll_lift, roll_damping, roll_stiffness, roll_velocity, roll_from_yaw, roll_prop_drag;
    double pitch_lift, pitch_damping, pitch_stiffness, pitch_velocity;
    double yaw_tilt, yaw_drag, yaw_damping, yaw_from_roll;
    double vx_pitch, vx_tilt, vx_drag, vy_roll, vy_drag;
    double arm, pivot_height, roll_inertia, pitch_inertia, yaw_inertia, reaction_torque_ratio;
    double mass, throttle_max, servo_limit, ground_k, ground_c, friction, air_drag;
    double hull[POINTS][3];
} Params;

size_t rigid_param_count(void) { return sizeof(Params) / sizeof(double); }

// np.interp on the thrust table (count rows of throttle, thrust).
static double interp(double x, const double *table, size_t count) {
    if (x > table[2 * (count - 1)]) return table[2 * (count - 1) + 1];
    if (x < table[0]) return table[1];
    size_t low = 0, high = count - 1;
    while (high - low > 1) {
        size_t middle = (low + high) / 2;
        if (table[2 * middle] <= x) low = middle;
        else high = middle;
    }
    if (table[2 * high] <= x) low = high;
    if (low == count - 1 || table[2 * low] == x) return table[2 * low + 1];
    double slope = (table[2 * low + 3] - table[2 * low + 1]) / (table[2 * low + 2] - table[2 * low]);
    double value = slope * (x - table[2 * low]) + table[2 * low + 1];
    if (isnan(value)) {
        value = slope * (x - table[2 * low + 2]) + table[2 * low + 3];
        if (isnan(value) && table[2 * low + 1] == table[2 * low + 3]) value = table[2 * low + 1];
    }
    return value;
}

// airframe.py delayed: one side's sample k minus delay, interpolated and never past k.
static double delayed(const double *buffer, size_t size, size_t side, double delay) {
    long k = (long)size - 1;
    double x = (double)k - delay / DT;
    double low = floor(x);
    double fraction = x - low;
    long first = (long)low, second = (long)low + 1;
    if (second > k) second = k;
    first = first < 0 ? 0 : first > k ? k : first;
    second = second < 0 ? 0 : second;
    return buffer[2 * first + side] * (1 - fraction) + buffer[2 * second + side] * fraction;
}

static double clip(double x, double low, double high) { return fmin(fmax(x, low), high); }

static double sign(double x) { return x > 0 ? 1.0 : x < 0 ? -1.0 : x; }

// Body-to-world rotation matrix of quaternion (w, x, y, z).
static void rotation(const double *q, double r[3][3]) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    r[0][0] = 1 - 2 * (y * y + z * z), r[0][1] = 2 * (x * y - w * z), r[0][2] = 2 * (x * z + w * y);
    r[1][0] = 2 * (x * y + w * z), r[1][1] = 1 - 2 * (x * x + z * z), r[1][2] = 2 * (y * z - w * x);
    r[2][0] = 2 * (x * z - w * y), r[2][1] = 2 * (y * z + w * x), r[2][2] = 1 - 2 * (x * x + y * y);
}

// r v and its transpose.
static void apply(double r[3][3], const double *v, double *out) {
    for (int i = 0; i < 3; i++) out[i] = r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2];
}

static void apply_transpose(double r[3][3], const double *v, double *out) {
    for (int i = 0; i < 3; i++) out[i] = r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2];
}

static void cross(const double *a, const double *b, double *out) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static double norm3(const double *v) { return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

// One control tick for n envs. Per env: pitch_tilt, roll_thrust, inertia_scale [n]; inertia, push [n, 3]; rotor_drag [n, 2]; throttle, tilt_command [n, 2] this tick's commands; disturbance [n, 6] forces then torques. Updated in place: throttles, commands [n, history, 2]; pos, vel, omega, accel [n, 3]; quat [n, 4]; the airframe state rate [n, 3], att, body_vel, motor, horn, hvel, tilt, tvel [n, 2]; contact, contact_now [n]; touched [n] (bytes).
void rigid_step(size_t n, size_t history, const Params *p, const double *table, size_t count,
                const double *pitch_tilt, const double *roll_thrust, const double *inertia, const double *inertia_scale, const double *rotor_drag, const double *push,
                const double *throttle, const double *tilt_command, const double *disturbance,
                double *throttles, double *commands, double *pos, double *vel, double *quat, double *omega,
                double *rate, double *att, double *body_vel, double *motor, double *horn, double *hvel, double *tilt, double *tvel,
                double *accel, double *contact, double *contact_now, unsigned char *touched) {
    const double gain[2] = {1.0, p->right_gain};
    const double motor_delay[2] = {p->motor_delay, p->right_motor_delay}, servo_delay[2] = {p->servo_delay, p->right_servo_delay};
    const double motor_keep[2] = {exp(-DT / p->motor_tau), exp(-DT / p->right_motor_tau)};
    const double thrust_gain[2] = {1.0, p->right_thrust_gain};
    const double wn[2] = {p->servo_wn, p->right_servo_wn};
    const double w2[2] = {wn[0] * wn[0], wn[1] * wn[1]}, z2[2] = {2 * p->servo_zeta * wn[0], 2 * p->servo_zeta * wn[1]};
    const double c2 = p->link_wn * p->link_wn, cz = 2 * p->link_zeta * p->link_wn;
    const double half[2] = {p->left_play / 2, p->right_play / 2}, band[2] = {p->left_band, p->right_band};
    const double h = DT / SUB;
    for (size_t e = 0; e < n; e++) {
        double *throttle_buffer = throttles + 2 * history * e, *command_buffer = commands + 2 * history * e;
        double *ps = pos + 3 * e, *v = vel + 3 * e, *q = quat + 4 * e, *w = omega + 3 * e;
        double *st_rate = rate + 3 * e, *st_att = att + 2 * e, *st_vel = body_vel + 2 * e;
        double *st_motor = motor + 2 * e, *st_horn = horn + 2 * e, *st_hvel = hvel + 2 * e, *st_tilt = tilt + 2 * e, *st_tvel = tvel + 2 * e;
        const double *force_disturbance = disturbance + 6 * e, *torque_disturbance = disturbance + 6 * e + 3;
        // The command history moves one sample on.
        memmove(throttle_buffer, throttle_buffer + 2, 2 * (history - 1) * sizeof(double));
        memmove(command_buffer, command_buffer + 2, 2 * (history - 1) * sizeof(double));
        for (int s = 0; s < 2; s++) {
            throttle_buffer[2 * (history - 1) + s] = clip(throttle[2 * e + s], 0, p->throttle_max);
            command_buffer[2 * (history - 1) + s] = clip(tilt_command[2 * e + s], -p->servo_limit, p->servo_limit) * (180.0 / 3.14159265358979323846);
        }
        // The airframe's inputs from the body's rates, tilt and speed now.
        double r0[3][3];
        rotation(q, r0);
        double *up = r0[2];
        double rate0[3] = {w[0], w[1], w[2]};
        double att0[2] = {clip(atan2(up[1], up[2]), -0.6, 0.6), clip(atan2(-up[0], hypot(up[1], up[2])), -0.6, 0.6)};
        double rotated[3];
        apply_transpose(r0, v, rotated);
        double vel0[2] = {rotated[0], rotated[1]};
        double flying = contact_now[e] == 0 ? 1.0 : 0.0;
        // airframe.py step.
        double target[2], goal[2], tl[2], c[2], sn[2];
        for (int s = 0; s < 2; s++) {
            target[s] = interp(delayed(throttle_buffer, history, s, motor_delay[s]), table, count) * 1.0 * thrust_gain[s];
            st_motor[s] = target[s] + (st_motor[s] - target[s]) * motor_keep[s];
            goal[s] = delayed(command_buffer, history, s, servo_delay[s]);
            tl[s] = st_tilt[s] * gain[s] * DEG;
            c[s] = cos(tl[s]), sn[s] = sin(tl[s]);
        }
        double left = st_motor[0], right = st_motor[1];
        double roll = roll_thrust[e] * p->arm / p->roll_inertia * (left * c[0] - right * c[1])
            + p->roll_lift * flying * p->arm / p->roll_inertia * (left * c[0] + right * c[1])
            + p->roll_prop_drag * p->reaction_torque_ratio / p->roll_inertia * (left * sn[0] - right * sn[1])
            + p->roll_damping * rate0[0] + p->roll_stiffness * flying * att0[0] + p->roll_velocity * flying * vel0[1] + p->roll_from_yaw * rate0[2]
            + p->roll_gyro * (st_tvel[0] - st_tvel[1]) * DEG
            + push[3 * e] * flying;
        double pitch_other = pitch_tilt[e] * (1 + 0.0) * p->pivot_height / p->pitch_inertia * (left * sn[0] + right * sn[1])
            + p->pitch_lift * (left * c[0] + right * c[1]) / p->pitch_inertia
            + p->pitch_damping * rate0[1] + p->pitch_stiffness * flying * att0[1] + p->pitch_velocity * flying * vel0[0]
            + push[3 * e + 1] * flying;
        double yaw = p->yaw_tilt * p->arm / p->yaw_inertia * (right * sn[1] - left * sn[0])
            + p->yaw_drag * (right * c[1] - left * c[0]) / p->yaw_inertia
            + p->yaw_damping * rate0[2] + p->yaw_from_roll * rate0[0]
            + push[3 * e + 2] * flying;
        double pq = rate0[1], pitch = att0[1], pitch_sum = 0.0;
        for (int sub = 0; sub < SUB; sub++) {
            double link[2];
            for (int s = 0; s < 2; s++) {
                st_hvel[s] = clip(st_hvel[s] + (w2[s] * (goal[s] - st_horn[s]) - z2[s] * st_hvel[s]) * h, -p->servo_slew, p->servo_slew);
                st_horn[s] = st_horn[s] + st_hvel[s] * h;
                double gap = st_horn[s] - st_tilt[s];
                gap = sign(gap) * fmax(fabs(gap) - half[s], 0);
                link[s] = c2 * gap + cz * (st_hvel[s] - st_tvel[s]) * (gap != 0 ? 1.0 : 0.0) + band[s] - p->band_stiffness * st_tilt[s];
            }
            double body = pitch_other - p->inertia_ratio * (link[0] * DEG + link[1] * DEG);
            for (int s = 0; s < 2; s++) {
                st_tvel[s] = st_tvel[s] + (link[s] - body / DEG) * h;
                st_tilt[s] = st_tilt[s] + st_tvel[s] * h;
            }
            pq = pq + body * h;
            pitch = pitch + pq * h;
            pitch_sum += body;
        }
        double alpha_model[3] = {roll, pitch_sum / SUB, yaw};
        st_rate[0] = rate0[0] + roll * DT, st_rate[1] = pq, st_rate[2] = rate0[2] + yaw * DT;
        st_att[0] = att0[0] + st_rate[0] * DT, st_att[1] = pitch;
        double mean_tilt = (st_tilt[0] * gain[0] + st_tilt[1] * gain[1]) / 2 * DEG;
        st_vel[0] = vel0[0] + (p->vx_pitch * att0[1] + p->vx_tilt * mean_tilt - p->vx_drag * vel0[0]) * DT;
        st_vel[1] = vel0[1] + (p->vy_roll * att0[0] - p->vy_drag * vel0[1]) * DT;
        // The rigid body.
        // The disturbances come with the rotors' wash: scaled by the thrust over the weight.
        double wash = clip((left + right) / (p->mass * G), 0, 1);
        double alpha[3], moment[3], pushed[3], drag_scale[2] = {rotor_drag[2 * e], rotor_drag[2 * e + 1]};
        for (int i = 0; i < 3; i++) {
            pushed[i] = force_disturbance[i] * wash;
            alpha[i] = alpha_model[i] / inertia_scale[e] + torque_disturbance[i] * wash;
            moment[i] = inertia[3 * e + i];
        }
        double sl, sr, cl, cr;
        {
            double left_tilt = st_tilt[0] * 1.0 * 3.14159265358979323846 / 180, right_tilt = st_tilt[1] * p->right_gain * 3.14159265358979323846 / 180;
            sl = sin(left_tilt), sr = sin(right_tilt), cl = cos(left_tilt), cr = cos(right_tilt);
        }
        double thrust_body[3] = {left * sl + right * sr, 0.0, left * cl + right * cr};
        double peak = 0.0;
        unsigned char touch = touched[e];
        for (int sub = 0; sub < SUB; sub++) {
            double spin[3], gyroscopic[3], r[3][3];
            for (int i = 0; i < 3; i++) spin[i] = moment[i] * w[i];
            cross(w, spin, gyroscopic);
            for (int i = 0; i < 3; i++) gyroscopic[i] = -gyroscopic[i] / moment[i];
            rotation(q, r);
            // Ground contact at the hull points.
            double contact_force[3] = {0, 0, 0}, contact_torque[3] = {0, 0, 0}, magnitude = 0.0;
            for (int k = 0; k < POINTS; k++) {
                double point[3], arm[3], lever[3], moving[3], point_vel[3], force[3], torque[3];
                const double *hull = p->hull[k];
                double depth = -((r[2][0] * hull[0] + r[2][1] * hull[1] + r[2][2] * hull[2]) + ps[2]);
                // A point off the ground adds nothing (zero force, torque and magnitude).
                if (!(depth > 0)) continue;
                touch = 1;
                apply(r, hull, point);
                for (int i = 0; i < 3; i++) point[i] = point[i] + ps[i];
                cross(w, hull, lever);
                apply(r, lever, moving);
                for (int i = 0; i < 3; i++) point_vel[i] = v[i] + moving[i];
                double normal = fmax(0.0, p->ground_k * depth - p->ground_c * point_vel[2]);
                double slip = sqrt(point_vel[0] * point_vel[0] + point_vel[1] * point_vel[1]);
                force[0] = -p->friction * normal * point_vel[0] / fmax(slip, SLIP);
                force[1] = -p->friction * normal * point_vel[1] / fmax(slip, SLIP);
                force[2] = normal;
                for (int i = 0; i < 3; i++) arm[i] = point[i] - ps[i];
                cross(arm, force, torque);
                for (int i = 0; i < 3; i++) contact_force[i] += force[i], contact_torque[i] += torque[i];
                magnitude += norm3(force);
            }
            double contact_alpha[3];
            apply_transpose(r, contact_torque, contact_alpha);
            for (int i = 0; i < 3; i++) contact_alpha[i] = contact_alpha[i] / moment[i];
            peak = fmax(peak, magnitude);
            // Thrust, drag and gravity.
            double v_body[3], force_body[3], force_world[3], accel_world[3], speed = norm3(v);
            apply_transpose(r, v, v_body);
            force_body[0] = thrust_body[0] + -(drag_scale[0] * v_body[0] * p->mass);
            force_body[1] = thrust_body[1] + -(drag_scale[1] * v_body[1] * p->mass);
            force_body[2] = thrust_body[2] + -0.0;
            apply(r, force_body, force_world);
            for (int i = 0; i < 3; i++) {
                force_world[i] = force_world[i] - p->air_drag * speed * v[i] + contact_force[i];
                accel_world[i] = force_world[i] / p->mass + pushed[i] - (i == 2 ? G : 0.0);
            }
            double specific[3] = {accel_world[0] + 0.0, accel_world[1] + 0.0, accel_world[2] + G};
            apply_transpose(r, specific, accel + 3 * e);
            for (int i = 0; i < 3; i++) v[i] = v[i] + accel_world[i] * h;
            for (int i = 0; i < 3; i++) ps[i] = ps[i] + v[i] * h;
            for (int i = 0; i < 3; i++) w[i] = w[i] + (alpha[i] + gyroscopic[i] + contact_alpha[i]) * h;
            // q advanced by the body rates (exact for constant rates), then normalized.
            double rate_norm = norm3(w), angle = rate_norm * h, axis_scale = fmax(rate_norm, 1e-9);
            double half_angle = angle / 2, sine = sin(half_angle);
            double turn[4] = {cos(half_angle), w[0] / axis_scale * sine, w[1] / axis_scale * sine, w[2] / axis_scale * sine};
            double w1 = q[0], x1 = q[1], y1 = q[2], z1 = q[3], w2q = turn[0], x2 = turn[1], y2 = turn[2], z2q = turn[3];
            double next[4] = {w1 * w2q - x1 * x2 - y1 * y2 - z1 * z2q, w1 * x2 + x1 * w2q + y1 * z2q - z1 * y2, w1 * y2 - x1 * z2q + y1 * w2q + z1 * x2, w1 * z2q + x1 * y2 - y1 * x2 + z1 * w2q};
            double length = sqrt(next[0] * next[0] + next[1] * next[1] + next[2] * next[2] + next[3] * next[3]);
            for (int i = 0; i < 4; i++) q[i] = next[i] / length;
        }
        touched[e] = touch;
        contact[e] = fmax(contact[e], peak);
        contact_now[e] = peak;
    }
}
