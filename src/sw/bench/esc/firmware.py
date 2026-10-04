"""
Firmware MicroPython for ESC control
"""

ESC_PIN = 25

from machine import PWM, Pin

def set_throttle(throttle):
    """
    Set throttle from 0 to 1000 (1000–2000 us pulses).
    """
    pwm.duty_ns((1000 + throttle) * 1000)


if __name__ == "__main__":
    pwm = PWM(Pin(ESC_PIN), freq=50, duty_u16=0)
    pwm.duty_ns(1_000_000)

    # Receive one ASCII integer per line, e.g. "100\n" for 10% throttle.
    while True:
        try:
            set_throttle(int(input()))

        except ValueError:
            pass
