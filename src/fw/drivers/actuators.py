"""
Independent left/right servo and ESC outputs.
"""

from machine import PWM, Pin

PINS = {
    'left': {'servo': 14, 'esc': 25},
    'right': {'servo': 16, 'esc': 18},
}
SERVO_LIMIT = 60
SERVO_OFFSETS = {'left': -3, 'right': 0}
THROTTLE_MAX = 1000


def init():
    """
    Start both ESCs at zero throttle and center both servos.
    """
    global outputs, readings
    outputs = {}
    readings = {}
    for side, pins in PINS.items():
        outputs[side] = {kind: PWM(Pin(pin), freq=50, duty_u16=0)
                         for kind, pin in pins.items()}
        readings[side] = {}
        set_throttle(side, 0)
        set_servo(side, 0)


def set_throttle(side, value):
    """
    Set throttle from 0 to 1000 (1000–2000 us pulses).
    """
    if not 0 <= value <= THROTTLE_MAX:
        raise ValueError('Throttle must be between 0 and 1000')
    outputs[side]['esc'].duty_ns(round((1000 + value) * 1000))
    readings[side]['throttle'] = value


def set_servo(side, angle):
    """
    Set servo degrees relative to its calibrated center.
    """
    if not -SERVO_LIMIT <= angle <= SERVO_LIMIT:
        raise ValueError('Servo angle must be between -60 and 60')
    outputs[side]['servo'].duty_ns(
        round(1_500_000 + (angle + SERVO_OFFSETS[side]) * 1_000_000 / 90)
    )
    readings[side]['servo_angle'] = angle


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
