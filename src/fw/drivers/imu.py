"""
MPU6050 on GPIO19 (SDA), GPIO22 (SCL), GPIO21 (INT), AD0 grounded.
"""

import asyncio
import math
import struct
import time
from machine import I2C, Pin
from drivers import control

MAHONY_KP = 1.0
MAHONY_KI = 0.3


def init():
    """
    Configure ±2 g acceleration and ±250 degrees/s angular velocity.
    """
    global bus, online, interrupt, ready, latest, last_read_ms
    global quaternion, integral, last_sample_us
    bus = I2C(0, sda=Pin(19), scl=Pin(22), freq=400_000)
    quaternion = None
    integral = [0.0, 0.0, 0.0]
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


def data_ready(pin):
    """
    Wake the sampling task without doing I2C work in the interrupt handler.
    """
    ready.set()


def update_roll(accel, gyro, now):
    """
    Mahony IMU filter: body-to-world quaternion, roll around sensor X in degrees.
    """
    global quaternion, last_sample_us
    ax, ay, az = accel
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if quaternion is None:
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        quaternion = (cr * cp, sr * cp, cr * sp, -sr * sp)
        last_sample_us = now

    dt = time.ticks_diff(now, last_sample_us) / 1_000_000
    last_sample_us = now
    w, x, y, z = quaternion
    gx, gy, gz = (v * math.pi / 180 for v in gyro)
    if norm > 0:
        ax, ay, az = ax / norm, ay / norm, az / norm
        # Predicted gravity in sensor coordinates; measured cross predicted error.
        vx, vy, vz = 2 * (x * z - w * y), 2 * (w * x + y * z), 1 - 2 * (x * x + y * y)
        ex, ey, ez = ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx
        for i, error in enumerate((ex, ey, ez)):
            integral[i] += MAHONY_KI * error * dt
        gx += MAHONY_KP * ex
        gy += MAHONY_KP * ey
        gz += MAHONY_KP * ez
    gx = (gx + integral[0]) * dt / 2
    gy = (gy + integral[1]) * dt / 2
    gz = (gz + integral[2]) * dt / 2
    qw = w - x * gx - y * gy - z * gz
    qx = x + w * gx + y * gz - z * gy
    qy = y + w * gy - x * gz + z * gx
    qz = z + w * gz + x * gy - y * gx
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    w, x, y, z = quaternion = (qw / norm, qx / norm, qy / norm, qz / norm)
    return math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / math.pi


async def sample():
    """
    Read a fresh sample when GPIO21 signals data ready.
    """
    global latest, last_read_ms, quaternion
    if not online:
        return
    while True:
        await ready.wait()
        try:
            # Read INT_STATUS and all axes together, clearing the latched INT.
            data = bus.readfrom_mem(0x68, 0x3A, 15)
        except OSError:
            control.stop()
            latest = None
            quaternion = None
            integral[:] = [0.0, 0.0, 0.0]
            continue
        ax, ay, az, temperature, gx, gy, gz = struct.unpack(">7h", data[1:])
        accel = [ax / 16384, ay / 16384, az / 16384]
        gyro = [gx / 131, gy / 131, gz / 131]
        latest = {
            "accel": accel,
            "gyro": gyro,
            "roll": update_roll(accel, gyro, time.ticks_us()),
            "temperature": temperature / 340 + 36.53,
        }
        control.update(latest["roll"], last_sample_us)
        last_read_ms = time.ticks_ms()
        # A pending interrupt makes ready.wait() return immediately.
        await asyncio.sleep_ms(0)


def read():
    """
    Return the latest sample, or None when interrupts have stopped arriving.
    """
    if time.ticks_diff(time.ticks_ms(), last_read_ms) > 1000:
        return None
    return latest
