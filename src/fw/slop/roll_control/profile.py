"""
ESP-side timing measurement; both ESCs stay at zero throttle.
"""

import time
from machine import WDT
from drivers import actuators, control

control.stop()
watchdog = WDT(timeout=2000)
for name, call in (
    ("PID math", lambda: control.calculate(0, time.ticks_us(), 200)),
    ("left PWM", lambda: actuators.set_throttle("left", 0)),
    ("right PWM", lambda: actuators.set_throttle("right", 0)),
):
    started = time.ticks_us()
    for _ in range(20):
        call()
    print(name, time.ticks_diff(time.ticks_us(), started) / 20, "us/call")
    watchdog.feed()
control.stop()
