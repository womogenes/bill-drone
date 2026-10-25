#ifndef ARMING_H
#define ARMING_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Wrap-safe sequence order, as the radio orders target packets: a is newer than b.
static inline bool sequence_newer(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) > 0;
}

// The highest laptop sequence of an accepted STOP since boot: an ARM packet must be newer. A reordered older STOP still stops the outputs but never lowers the barrier.
typedef struct {
    bool set;
    uint32_t sequence;
} StopBarrier;

static inline void stop_barrier_update(StopBarrier *barrier, uint32_t sequence) {
    if (!barrier->set || sequence_newer(sequence, barrier->sequence)) {
        barrier->set = true;
        barrier->sequence = sequence;
    }
}

// Everything arming depends on, read under the runtime lock in one go (flight_rt.c flight_runtime_arm).
typedef struct {
    bool running, online, calibrated, calibrating, enabled, throttle, base, recorder, log_pending;
    // A finished calibration is not trusted until it is committed to NVS: ready (not yet taken by the update task) and unsaved (taken, not yet committed) both refuse.
    bool calibration_ready, calibration_unsaved;
    // An NVS or flash write in progress; outputs are held at zero meanwhile.
    bool maintenance;
    // Stops counted when the arm request arrived and now: any stop in between refuses the request.
    uint32_t expected_generation, generation;
    // The arm packet's sequence must also be newer than every accepted STOP's, so reordering cannot sneak it past.
    uint32_t arm_sequence;
    StopBarrier barrier;
} ArmCheck;

// Why arming is refused, or NULL.
static inline const char *arm_refusal(const ArmCheck *check) {
    if (check->generation != check->expected_generation) return "Stopped after arm request";
    if (check->barrier.set && !sequence_newer(check->arm_sequence, check->barrier.sequence)) return "Stopped after arm request";
    if (!check->running || !check->online) return "IMU offline";
    if (!check->calibrated || check->calibrating) return "Calibrate the IMU before arming";
    if (check->calibration_ready || check->calibration_unsaved) return "Calibration not saved yet";
    if (check->maintenance) return "Saving to flash; try again";
    if (check->enabled) return "Already armed";
    if (check->base || check->throttle) return "Set throttle to zero before arming";
    if (!check->recorder) return "Flight recorder is not ready";
    if (check->log_pending) return "Previous recording is still sending";
    return NULL;
}

// Recently handled arm request ids: a repeated id replays its first result and never arms again, even after the reply cache has moved on to other requests.
#define ARM_HISTORY 8
typedef struct {
    uint8_t id[ARM_HISTORY][4];
    bool used[ARM_HISTORY], armed[ARM_HISTORY];
    int next;
} ArmHistory;

// -1 for a new id, otherwise whether that id armed.
static inline int arm_history_find(const ArmHistory *history, const uint8_t *id) {
    for (int i = 0; i < ARM_HISTORY; i++) {
        if (history->used[i] && !memcmp(history->id[i], id, 4)) return history->armed[i];
    }
    return -1;
}

static inline void arm_history_add(ArmHistory *history, const uint8_t *id, bool armed) {
    memcpy(history->id[history->next], id, 4);
    history->used[history->next] = true;
    history->armed[history->next] = armed;
    history->next = (history->next + 1) % ARM_HISTORY;
}

#endif
