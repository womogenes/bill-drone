"""
HX711 channel A, gain 128. Read raw counts without waiting for a conversion.
Wiring: DT/DOUT to GPIO32, SCK to GPIO33, VCC to 3V3, GND to GND.
"""

from machine import Pin, disable_irq, enable_irq
import micropython
from time import sleep_us


class LoadCell:
    def __init__(self, dout=32, sck=33):
        self.sck = Pin(sck, Pin.OUT, value=0)
        self.dout = Pin(dout, Pin.IN)
        self.value = None

    @micropython.native
    def read(self):
        """
        Return the latest signed raw count, or None before the first sample.

        Call regularly to collect samples. If DOUT is high, the HX711 is still
        converting and the previous reading is returned unchanged.
        """
        dout, sck = self.dout, self.sck
        if dout():
            return self.value

        value = 0
        # A clock-high pause over 60 us powers down the HX711. Keep this short
        # transfer uninterrupted; no conversion waiting occurs with IRQs off.
        irq = disable_irq()
        try:
            for _ in range(24):
                sck(1)
                sleep_us(1)
                bit = dout()
                sck(0)
                value = (value << 1) | bit
                sleep_us(1)

            # The 25th pulse selects channel A / gain 128 for the next sample.
            sck(1)
            sleep_us(1)

        finally:
            sck(0)
            enable_irq(irq)

        if value & 0x800000:
            value -= 0x1000000

        self.value = value
        return value


if __name__ == "__main__":
    # Test manually
    load_cell = LoadCell()

    import time
    while True:
        print(load_cell.read())
        time.sleep(0.15)
