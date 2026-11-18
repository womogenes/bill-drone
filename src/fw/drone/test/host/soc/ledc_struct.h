#pragma once

#include <stdint.h>

// The duty registers the control task writes (flight_rt.c pulse), as plain fields the test reads after every operation.
typedef struct {
    struct {
        struct {
            struct { uint32_t duty_start; } conf1;
            struct { uint32_t duty; } duty;
        } channel[8];
    } channel_group[2];
} ledc_dev_t;
extern ledc_dev_t LEDC;
