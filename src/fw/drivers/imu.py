"""
MPU6050 on GPIO21 (SDA), GPIO22 (SCL), AD0 grounded.
"""

import struct
import time
from machine import I2C, Pin


def init():
    """
    Configure ±2 g acceleration and ±250 degrees/s angular velocity.
    """
    global bus
    bus = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
    bus.writeto_mem(0x68, 0x6B, b'\x01')  # Wake, use X gyro clock.
    time.sleep_ms(100)
    bus.writeto_mem(0x68, 0x19, b'\x09\x03\x00\x00')  # 100 Hz, DLPF, gyro, accel.


def read():
    """
    Read all axes and temperature in one coherent I2C burst.
    """
    ax, ay, az, temperature, gx, gy, gz = struct.unpack(
        '>7h', bus.readfrom_mem(0x68, 0x3B, 14)
    )
    return {
        'accel': [ax / 16384, ay / 16384, az / 16384],
        'gyro': [gx / 131, gy / 131, gz / 131],
        'temperature': temperature / 340 + 36.53,
    }
