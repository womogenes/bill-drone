"""
Drone application: load cell, battery, recording metadata and file updates. The firmware's native runtime runs the radio link, streamed targets, stop, the motor watchdog and control (native/runtime/radio.c and flight_rt.c). It boots at 240 MHz; changing the CPU frequency here would stall the radio.
"""

from machine import ADC, Pin
import asyncio
import time
import flight_rt
import radio
import settings

from drivers import actuators, direct, imu, migrate, protocol
from drivers.load_cell import LoadCell

migrate.run()
actuators.init()
load_cell = LoadCell()
imu.init()

adc = ADC(Pin(35), atten=ADC.ATTN_6DB)
adc.block().init(bits=12)
VOLTAGE_DIVIDER_RATIO = (100_000 + 10_000) / 10_000
STATUS_MS = 50


async def report():
    """
    Fill the application part of the drone's status message and keep the recording metadata current.
    """
    described = None
    metadata = None
    while True:
        started = time.ticks_ms()
        if described != settings.version():
            described = settings.version()
            metadata = protocol.metadata()
        # Refused while the previous metadata is still being sent; the same bytes are offered again.
        if metadata is not None and radio.log_meta(metadata):
            metadata = None
        force = load_cell.read()
        voltage = adc.read_uv() * VOLTAGE_DIVIDER_RATIO / 1_000_000
        flight_rt.set_voltage(round(voltage * 1000))
        age = time.ticks_diff(time.ticks_ms(), load_cell.last_read_ms) if load_cell.last_read_ms is not None else 0
        radio.status(protocol.app_status(force, age, load_cell.sequence, adc.read_u16(), voltage))
        await asyncio.sleep_ms(max(0, STATUS_MS - time.ticks_diff(time.ticks_ms(), started)))


async def main():
    try:
        await asyncio.gather(report(), direct.requests())
    finally:
        flight_rt.stop(1)
        imu.stop()


if __name__ == "__main__":
    print("Starting app...")
    asyncio.run(main())
