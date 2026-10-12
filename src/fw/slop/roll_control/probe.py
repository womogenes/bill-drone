"""
Temporary onboard experiment. Never imported by boot.py or main.py.
"""

import asyncio
from array import array
import json
import gc
import time
from machine import WDT
from drivers import actuators, control, imu


async def run(kp, ki, kd, base=200, runid="", seconds=8):
    actuators.init()
    watchdog = WDT(timeout=2000)
    gc.collect()
    data = array("f", (0 for _ in range(600 * 6)))
    count = 0
    started = time.ticks_us()
    initial = None
    centered = False
    reason = "duration"
    control_us = 0
    control_count = 0
    original = control.update
    finished = asyncio.Event()

    def sample(roll, now):
        nonlocal count, initial, centered, reason, control_us, control_count
        elapsed = time.ticks_diff(now, started) / 1e6
        if initial is None:
            initial = roll
            control.configure(
                dict(enabled=True, target=0, kp=kp, ki=ki, kd=kd, base_throttle=base)
            )
        centered |= abs(roll) < 30
        if (centered and abs(roll) > 30) or (
            not centered and abs(roll) > abs(initial) + 0.5
        ):
            reason = "angle boundary"
            control.stop()
            finished.set()
        elif abs(imu.latest["gyro"][0]) > 180:
            reason = "rate boundary"
            control.stop()
            finished.set()
        elif elapsed > seconds or count >= 600:
            control.stop()
            finished.set()
        elif not centered and elapsed > 4:
            reason = "startup timeout"
            control.stop()
            finished.set()
        else:
            before = time.ticks_us()
            original(roll, now)
            control_us += time.ticks_diff(time.ticks_us(), before)
            control_count += 1
        if count < 600:
            outputs = actuators.read()
            for j, v in enumerate(
                (
                    elapsed,
                    roll,
                    imu.latest["gyro"][0],
                    outputs["left"]["throttle"],
                    outputs["right"]["throttle"],
                    control.integral,
                )
            ):
                data[count * 6 + j] = v
            count += 1

    control.update = sample
    imu.ready = asyncio.ThreadSafeFlag()
    imu.last_sample_us = time.ticks_us()
    imu.ready.set()
    task = asyncio.create_task(imu.sample())
    try:
        while not finished.is_set():
            await asyncio.sleep_ms(20)
            watchdog.feed()
            if (
                time.ticks_diff(time.ticks_us(), started) > 200000
                and time.ticks_diff(time.ticks_ms(), imu.last_read_ms) > 100
            ):
                reason = "IMU timeout"
                break
    finally:
        control.stop()
        task.cancel()
        control.update = original
        with open("roll_probe.bin", "wb") as f:
            f.write(memoryview(data)[: count * 6])
        with open("roll_probe.json", "w") as f:
            json.dump(
                dict(
                    kp=kp,
                    ki=ki,
                    kd=kd,
                    base=base,
                    runid=runid,
                    reason=reason,
                    count=count,
                    control_us=control_us,
                    control_count=control_count,
                ),
                f,
            )
        print("PROBE DONE", reason, count)
