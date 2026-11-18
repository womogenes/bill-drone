// The firmware's flight math (fw/drone/main/flight_math.c) and INDI (indi.c) for the host.
#include "flight_math.c"
#include "indi.c"

// Where Command's runtime-only fields that the host sets by name sit, in floats from its start.
const size_t flight_command_indi = offsetof(Command, indi_share) / sizeof(float);
const size_t flight_command_attitude = offsetof(Command, attitude) / sizeof(float);
const size_t flight_command_size = sizeof(Command) / sizeof(float);
