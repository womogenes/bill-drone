"""
Independent left/right servo and ESC outputs.
"""

from machine import PWM, Pin, mem32
import flight_rt

PINS = {
    "left": {"servo": 14, "esc": 25},
    "right": {"servo": 16, "esc": 18},
}
SIDES = ("left", "right")
running = False
SERVO_LIMIT = 60
SERVO_OFFSETS = {"left": -3, "right": 0}
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
    global outputs, readings, pwm_registers

    outputs = {}
    readings = {}
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

        readings[side] = {}
        set_throttle(side, 0)
        set_servo(side, 0)


def set_pulse(side, kind, pulse_us):
    """
    Latch a PWM duty update without waiting for the previous transition.
    """
    register, ticks = pwm_registers[side][kind]
    mem32[register + 12] &= 0x7FFFFFFF
    mem32[register + 8] = round(pulse_us * ticks * PWM_HZ[kind] / 1_000_000) << 4
    mem32[register + 12] |= 1 << 31


def set_throttle(side, value):
    """
    Set throttle from 0 to 1200 (1000–2200 us pulses).
    """
    if not 0 <= value <= THROTTLE_MAX:
        raise ValueError(f"Throttle must be between 0 and {THROTTLE_MAX}")

    if running:
        flight_rt.manual(SIDES.index(side), 1, value)
    else:
        set_pulse(side, "esc", 1000 + value)
    readings[side]["throttle"] = value


def set_servo(side, angle):
    """
    Set servo degrees relative to its calibrated center.
    """
    if not -SERVO_LIMIT <= angle <= SERVO_LIMIT:
        raise ValueError("Servo angle must be between -60 and 60")

    if running:
        flight_rt.manual(SIDES.index(side), 0, angle)
    else:
        set_pulse(side, "servo", 1500 + (angle + SERVO_OFFSETS[side]) * 1000 / 90)
    readings[side]["servo_angle"] = angle


def zero_servo(side):
    """
    Keep the current pulse and make its commanded angle the new zero.
    """
    if running:
        SERVO_OFFSETS[side] = flight_rt.zero_servo(SIDES.index(side))
    else:
        SERVO_OFFSETS[side] += readings[side]["servo_angle"]
    readings[side]["servo_angle"] = 0


def stop():
    """
    Stop both ESCs.
    """
    for side in outputs:
        set_throttle(side, 0)


def read(snapshot=None):
    """
    Return commanded positions and throttles for telemetry.
    """
    if running:
        state = snapshot if snapshot is not None else flight_rt.snapshot()
        for index, side in enumerate(SIDES):
            readings[side]["servo_angle"] = state["servo"][index]
            readings[side]["throttle"] = state["throttle"][index]
    return readings


def native_pwm():
    return [
        (pwm_registers[side][kind][0], pwm_registers[side][kind][1] * PWM_HZ[kind] / 1_000_000)
        for side in SIDES
        for kind in ("servo", "esc")
    ]
