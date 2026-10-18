"""
MPU6050 on GPIO19 (SDA), GPIO22 (SCL), GPIO21 (INT), AD0 grounded.
"""

import asyncio
import math
import struct
import time
from machine import I2C, Pin
from drivers import control, actuators
from drivers.flight import Attitude
from drivers.calibration import GyroCalibration

CALIBRATION_SAMPLES = 200


def init():
    """
    Configure 100 Hz sampling, DLPF setting 3, ±2 g, and ±250 degrees/s.
    """
    global bus, online, interrupt, ready, latest, last_read_ms
    global estimator, calibration, last_sample_us, calibration_count

    bus = I2C(0, sda=Pin(19), scl=Pin(22), freq=400_000)

    calibration = GyroCalibration()
    calibration_count = 0
    estimator = Attitude()
    last_sample_us = None

    online = False
    ready = asyncio.ThreadSafeFlag()
    latest = None
    last_read_ms = 0

    interrupt = Pin(21, Pin.IN, Pin.PULL_DOWN)
    interrupt.irq(handler=data_ready, trigger=Pin.IRQ_RISING)

    try:
        bus.writeto_mem(0x68, 0x6B, b"\x01")  # Wake, use X gyro clock.
        time.sleep_ms(100)

        bus.writeto_mem(0x68, 0x19, b"\x09\x03\x00\x00")  # 100 Hz, DLPF, gyro, accel.
        bus.writeto_mem(0x68, 0x37, b"\x20")  # Active-high, latched until status read.
        bus.readfrom_mem(0x68, 0x3A, 1)
        bus.writeto_mem(0x68, 0x38, b"\x01")  # Enable data-ready interrupt.
        online = True

    except OSError:
        pass


def calibrate():
    """
    Stop the motors and average 200 stationary gyro samples to estimate bias.
    """
    global calibration_count

    control.stop()
    calibration.start(CALIBRATION_SAMPLES)
    calibration_count = CALIBRATION_SAMPLES


def data_ready(pin):
    """
    Wake the sampling task without doing I2C work in the interrupt handler.
    """
    ready.set()


async def sample():
    """
    Read a fresh sample when GPIO21 signals data ready.
    """
    global latest, last_read_ms, estimator, last_sample_us, calibration_count

    if not online:
        return

    while True:
        await ready.wait()

        try:
            # Read INT_STATUS and all axes together, clearing the latched INT.
            data = bus.readfrom_mem(0x68, 0x3A, 15)

        except OSError:
            control.stop()
            calibration_count = 0
            latest = None
            estimator = Attitude()
            last_sample_us = None
            calibration.remaining = 0
            continue

        ax, ay, az, temperature, gx, gy, gz = struct.unpack(">7h", data[1:])
        # Verified by nose-lift test: sensor X back, Y left, Z down.
        accel = [-ax / 16384, ay / 16384, -az / 16384]
        gyro = [-gx / 131, gy / 131, -gz / 131]

        gyro = calibration.update([math.radians(value) for value in gyro])
        calibration_count = calibration.remaining
        now = time.ticks_us()
        if last_sample_us is None:
            estimator = Attitude(accel)

        dt = time.ticks_diff(now, last_sample_us) / 1_000_000 if last_sample_us is not None else 0.01
        last_sample_us = now
        powered = any(value["throttle"] > 0 for value in actuators.read().values())
        roll, pitch = estimator.update(accel, gyro, dt, powered)
        roll, pitch = math.degrees(roll), math.degrees(pitch)
        gyro = [math.degrees(value) for value in gyro]

        latest = {
            "calibrating": calibration_count > 0,
            "accel": accel,
            "gyro": gyro,
            "roll": roll,
            "pitch": pitch,
            "temperature": temperature / 340 + 36.53,
        }

        control.update(latest, last_sample_us)
        last_read_ms = time.ticks_ms()

        # A pending interrupt makes ready.wait() return immediately.
        await asyncio.sleep_ms(0)


def read():
    """
    Return the latest sample, or None if it is more than one second old.
    """
    if time.ticks_diff(time.ticks_ms(), last_read_ms) > 1000:
        return None

    return latest
