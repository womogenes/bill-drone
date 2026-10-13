"""
Independent left/right servo and ESC outputs.
"""

from machine import PWM, Pin, mem32

PINS = {
    "left": {"servo": 14, "esc": 25},
    "right": {"servo": 16, "esc": 18},
}
SERVO_LIMIT = 60
SERVO_OFFSETS = {"left": -3, "right": 0}
THROTTLE_MAX = 1200
PWM_HZ = 50


def init():
    """
    Start both ESCs at zero throttle and center both servos.
    """
    global outputs, readings, pwm_registers

    outputs = {}
    readings = {}
    pwm_registers = {}

    for side, pins in PINS.items():
        outputs[side] = {
            kind: PWM(Pin(pin), freq=PWM_HZ, duty_u16=0) for kind, pin in pins.items()
        }

        pwm_registers[side] = {}
        for kind, pin in pins.items():
            # Original ESP32: GPIO matrix selects LEDC high-speed channels 71..78.
            channel = (mem32[0x3FF44530 + 4 * pin] & 0x1FF) - 71
            assert 0 <= channel < 8, "PWM requires an ESP32 high-speed LEDC channel"
            register = 0x3FF59000 + 20 * channel
            timer = mem32[register] & 3
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
    mem32[register + 8] = round(pulse_us * ticks * PWM_HZ / 1_000_000) << 4
    mem32[register + 12] |= 1 << 31


def set_throttle(side, value):
    """
    Set throttle from 0 to 1200 (1000–2200 us pulses).
    """
    if not 0 <= value <= THROTTLE_MAX:
        raise ValueError(f"Throttle must be between 0 and {THROTTLE_MAX}")

    set_pulse(side, "esc", 1000 + value)
    readings[side]["throttle"] = value


def set_servo(side, angle):
    """
    Set servo degrees relative to its calibrated center.
    """
    if not -SERVO_LIMIT <= angle <= SERVO_LIMIT:
        raise ValueError("Servo angle must be between -60 and 60")

    set_pulse(side, "servo", 1500 + (angle + SERVO_OFFSETS[side]) * 1000 / 90)
    readings[side]["servo_angle"] = angle


def stop():
    """
    Stop both ESCs.
    """
    for side in outputs:
        set_throttle(side, 0)


def read():
    """
    Return commanded positions and throttles for telemetry.
    """
    return readings
