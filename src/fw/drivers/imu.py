"""
MPU6050 on GPIO19 (SDA), GPIO22 (SCL), GPIO21 (INT), AD0 grounded.
"""

import asyncio
import struct
import time
from machine import I2C, Pin


def init():
    """
    Configure ±2 g acceleration and ±250 degrees/s angular velocity.
    """
    global bus, online, interrupt, ready, latest, last_read_ms
    bus = I2C(0, sda=Pin(19), scl=Pin(22), freq=400_000)
    online = False
    ready = asyncio.ThreadSafeFlag()
    latest = None
    last_read_ms = 0
    interrupt = Pin(21, Pin.IN, Pin.PULL_DOWN)
    interrupt.irq(handler=_data_ready, trigger=Pin.IRQ_RISING)
    try:
        bus.writeto_mem(0x68, 0x6B, b'\x01')  # Wake, use X gyro clock.
        time.sleep_ms(100)
        bus.writeto_mem(0x68, 0x19, b'\x09\x03\x00\x00')  # 100 Hz, DLPF, gyro, accel.
        bus.writeto_mem(0x68, 0x37, b'\x20')  # Active-high, latched until status read.
        bus.readfrom_mem(0x68, 0x3A, 1)
        bus.writeto_mem(0x68, 0x38, b'\x01')  # Enable data-ready interrupt.
        online = True
    except OSError:
        pass


def _data_ready(pin):
    """
    Wake the sampling task without doing I2C work in the interrupt handler.
    """
    ready.set()


async def sample():
    """
    Read a fresh sample when GPIO21 signals data ready.
    """
    global latest, last_read_ms
    if not online:
        return
    while True:
        await ready.wait()
        try:
            # Read INT_STATUS and all axes together, clearing the latched INT.
            data = bus.readfrom_mem(0x68, 0x3A, 15)
        except OSError:
            latest = None
            continue
        ax, ay, az, temperature, gx, gy, gz = struct.unpack('>7h', data[1:])
        latest = {
            'accel': [ax / 16384, ay / 16384, az / 16384],
            'gyro': [gx / 131, gy / 131, gz / 131],
            'temperature': temperature / 340 + 36.53,
        }
        last_read_ms = time.ticks_ms()


def read():
    """
    Return the latest sample, or None when interrupts have stopped arriving.
    """
    if time.ticks_diff(time.ticks_ms(), last_read_ms) > 1000:
        return None
    return latest
