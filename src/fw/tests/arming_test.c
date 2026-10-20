// Host test of the native arming decision (native/runtime/arming.h) as flight_rt.c and radio.c use it:
// cc -DSETTINGS_HOST tests/arming_test.c -lm (make -C src/fw test).
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../native/runtime/arming.h"
#include "../native/runtime/settings.c"

// The runtime state arming reads, with every check passing; flight_rt.c fills the same fields under its lock.
static uint32_t generation;
static bool calibration_ready, maintenance;

static ArmCheck ready_to_arm(uint32_t expected) {
    return (ArmCheck){
        .running = true, .online = true, .calibrated = true, .recorder = true,
        .calibration_ready = calibration_ready, .calibration_unsaved = settings_calibration_unsaved(),
        .maintenance = maintenance, .expected_generation = expected, .generation = generation,
    };
}

// radio.c: a completed request records the stop count; a STOP increments it inside the runtime's stop.
static uint32_t receive_arm(void) {
    return generation;
}

static void receive_stop(void) {
    generation++;
}

// radio.c handle('A'): a repeated id replays its first result through the history and never reaches the arm.
static ArmHistory history;
static int arms;

static const char *handle_arm(const uint8_t *id, uint32_t expected) {
    int earlier = arm_history_find(&history, id);
    if (earlier >= 0) return earlier ? NULL : "Arm request was refused earlier";
    ArmCheck check = ready_to_arm(expected);
    const char *error = arm_refusal(&check);
    if (!error) arms++;
    arm_history_add(&history, id, !error);
    return error;
}

int main(void) {
    settings_decode(NULL, 0);

    // ARM received, then STOP, then the ARM handled: refused.
    uint32_t request = receive_arm();
    receive_stop();
    assert(!strcmp(handle_arm((const uint8_t *)"arm1", request), "Stopped after arm request") && !arms);

    // STOP, then ARM: allowed when everything else passes.
    receive_stop();
    request = receive_arm();
    assert(!handle_arm((const uint8_t *)"arm2", request) && arms == 1);

    // STOP between the request's validation and the runtime's arm: the comparison runs inside the arm, so refused.
    request = receive_arm();
    ArmCheck check = ready_to_arm(request);
    receive_stop();
    check.generation = generation;
    assert(!strcmp(arm_refusal(&check), "Stopped after arm request"));

    // Replayed ARM ids after a STOP, even recorded at the new stop count, return their first result and never arm.
    receive_stop();
    assert(!handle_arm((const uint8_t *)"arm2", receive_arm()) && arms == 1);
    assert(!strcmp(handle_arm((const uint8_t *)"arm1", receive_arm()), "Arm request was refused earlier") && arms == 1);
    for (int i = 0; i < ARM_HISTORY - 2; i++) handle_arm((const uint8_t[]){'x', 'y', 'z', (uint8_t)i}, generation - 1);
    assert(!handle_arm((const uint8_t *)"arm2", receive_arm()) && arms == 1);

    // A finished calibration: not taken yet, then taken but not committed, both refuse; committed allows.
    calibration_ready = true;
    check = ready_to_arm(generation);
    assert(!strcmp(arm_refusal(&check), "Calibration not saved yet"));
    calibration_ready = false;
    int index = SETTING_calibrated;
    float one = 1;
    settings_change(&index, &one, 1, 0);
    check = ready_to_arm(generation);
    assert(!strcmp(arm_refusal(&check), "Calibration not saved yet"));
    uint8_t blob[BLOB];
    settings_decode(blob, settings_encode(blob, sizeof(blob)));
    check = ready_to_arm(generation);
    assert(!settings_calibration_unsaved() && !arm_refusal(&check));

    // An uncalibrated IMU (any rejected stored calibration) and a flash write in progress both refuse.
    check = ready_to_arm(generation);
    check.calibrated = false;
    assert(!strcmp(arm_refusal(&check), "Calibrate the IMU before arming"));
    maintenance = true;
    check = ready_to_arm(generation);
    assert(!strcmp(arm_refusal(&check), "Saving to flash; try again"));
    maintenance = false;
    check = ready_to_arm(generation);
    check.throttle = true;
    assert(!strcmp(arm_refusal(&check), "Set throttle to zero before arming"));
    // Reordered packets: an ARM must be newer than every accepted STOP, wrap-safe.
    StopBarrier barrier = {0};
    check = ready_to_arm(generation);
    stop_barrier_update(&barrier, 10);
    check.barrier = barrier;
    check.arm_sequence = 9;
    assert(!strcmp(arm_refusal(&check), "Stopped after arm request"));
    check.arm_sequence = 10;
    assert(!strcmp(arm_refusal(&check), "Stopped after arm request"));
    check.arm_sequence = 11;
    assert(!arm_refusal(&check));
    // A reordered older STOP stops the outputs but keeps the barrier.
    stop_barrier_update(&barrier, 8);
    check.barrier = barrier;
    check.arm_sequence = 9;
    assert(barrier.sequence == 10 && !strcmp(arm_refusal(&check), "Stopped after arm request"));
    check.arm_sequence = 11;
    assert(!arm_refusal(&check));
    // Wraparound near 0xFFFFFFFF.
    barrier = (StopBarrier){0};
    stop_barrier_update(&barrier, 0xFFFFFFF0u);
    check.barrier = barrier;
    check.arm_sequence = 5;
    assert(!arm_refusal(&check));
    check.arm_sequence = 0xFFFFFFE0u;
    assert(!strcmp(arm_refusal(&check), "Stopped after arm request"));
    stop_barrier_update(&barrier, 3);
    check.barrier = barrier;
    check.arm_sequence = 2;
    assert(barrier.sequence == 3 && !strcmp(arm_refusal(&check), "Stopped after arm request"));
    // A stream of repeated STOPs, then a fresh newer ARM.
    barrier = (StopBarrier){0};
    for (uint32_t sequence = 100; sequence < 120; sequence++) stop_barrier_update(&barrier, sequence);
    check.barrier = barrier;
    check.arm_sequence = 121;
    assert(!arm_refusal(&check));
    // A new session (boot) starts without a barrier.
    check.barrier = (StopBarrier){0};
    check.arm_sequence = 1;
    assert(!arm_refusal(&check));
    printf("arming: stop ordering, packet reordering, replay, calibration save windows and maintenance refuse as expected\n");
    return 0;
}
