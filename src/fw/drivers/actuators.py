"""
Left/right servo and ESC outputs. The native runtime owns them once it starts; this sets up PWM and neutral pulses.
"""

from machine import PWM, Pin, mem32
import settings

PINS = {
    "left": {"servo": 14, "esc": 25},
    "right": {"servo": 16, "esc": 18},
}
SIDES = ("left", "right")
SERVO_LIMIT = 60
THROTTLE_MAX = 1200
# MG995 analog servos stay at the standard 50 Hz. ESC refresh is independent; 50 and 400 Hz both use 1000–2200 us pulses.
SERVO_PWM_HZ = 50
ESC_PWM_HZ = 400
PWM_HZ = {"servo": SERVO_PWM_HZ, "esc": ESC_PWM_HZ}
assert 1_000_000 / ESC_PWM_HZ >= 1000 + THROTTLE_MAX + 100, "ESC period must exceed the longest pulse"


def init():
    """
    Start both ESCs at zero throttle and center both servos.
    """
    global outputs, pwm_registers

    outputs = {}
    pwm_registers = {}
    timers = {}

    for side, pins in PINS.items():
        outputs[side] = {
            kind: PWM(Pin(pin), freq=PWM_HZ[kind], duty_u16=0) for kind, pin in pins.items()
        }

        pwm_registers[side] = {}
        for kind, pin in pins.items():
            # Original ESP32: GPIO matrix selects LEDC high-speed channels 71..78.
            channel = (mem32[0x3FF44530 + 4 * pin] & 0x1FF) - 71
            assert 0 <= channel < 8, "PWM requires an ESP32 high-speed LEDC channel"

            register = 0x3FF59000 + 20 * channel
            timer = mem32[register] & 3
            # Each LEDC timer has one frequency, so servo and ESC channels must not share one at different rates.
            assert timers.setdefault(timer, PWM_HZ[kind]) == PWM_HZ[kind], "Servo and ESC PWM share a timer"
            resolution = mem32[0x3FF59140 + 8 * timer] & 31
            pwm_registers[side][kind] = register, 1 << resolution

        set_pulse(side, "esc", 1000)
        set_pulse(side, "servo", 1500 + settings.get(f"servo_offset_{side}") * 1000 / 90)


def set_pulse(side, kind, pulse_us):
    """
    Latch a PWM duty update without waiting for the previous transition.
    """
    register, ticks = pwm_registers[side][kind]
    mem32[register + 12] &= 0x7FFFFFFF
    mem32[register + 8] = round(pulse_us * ticks * PWM_HZ[kind] / 1_000_000) << 4
    mem32[register + 12] |= 1 << 31


def native_pwm():
    return [
        (pwm_registers[side][kind][0], pwm_registers[side][kind][1] * PWM_HZ[kind] / 1_000_000)
        for side in SIDES
        for kind in ("servo", "esc")
    ]
