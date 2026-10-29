// The firmware's flight math (fw/drone/main/flight_math.c) for the host, plus many controllers stepped in one call for the identification (sw/sim/identify_airframe.py): lane i's state and command start i * state_stride and i * command_stride floats in.
#include "flight_math.c"

void flight_control_many(float *states, size_t state_stride, const float *commands, size_t command_stride, const float *table, size_t count, size_t lanes) {
    for (size_t i = 0; i < lanes; i++) flight_control((Controller *)(states + i * state_stride), (const Command *)(commands + i * command_stride), table, count);
}
