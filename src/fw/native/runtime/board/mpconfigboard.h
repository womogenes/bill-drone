#define MICROPY_HW_BOARD_NAME "Bill drone ESP32"
#define MICROPY_HW_MCU_NAME "ESP32"
#define MP_TASK_COREID (0)

void flight_runtime_stop(void);
#define MICROPY_BOARD_SOFT_RESET() flight_runtime_stop()
