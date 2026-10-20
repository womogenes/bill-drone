"""
USB-to-ESP-NOW relay. The laptop supplies every controller heartbeat.
"""

import espnow
import machine
import micropython
import network
import sys
import time
from wire import SerialDecoder, serial_encode, unpack
from controller_config import DRONE_MAC, CHANNEL

time.sleep(2)  # Allow Ctrl-C after reset for subsequent uploads.
machine.freq(240_000_000)
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.disconnect()
wifi.config(pm=network.WLAN.PM_NONE, channel=CHANNEL)
radio = espnow.ESPNow()
radio.config(rxbuf=4096)
radio.active(True)
radio.add_peer(DRONE_MAC)

# Raw UART owns USB while this program runs; EN restores the boot REPL window.
micropython.kbd_intr(-1)
uart = machine.UART(1, baudrate=921600, tx=1, rx=3, rxbuf=4096, txbuf=4096, timeout=0)
decoder = SerialDecoder()
outgoing = []

try:
    while True:
        if uart.any():
            for packet in decoder.feed(uart.read(512)):
                channel = unpack(packet)[0]
                if channel in (1, 4, 5):
                    if len(outgoing) == 8:
                        outgoing.pop(0)
                    outgoing.append(packet)

        sent, responses, failures, received, dropped = radio.stats()
        if outgoing and sent - responses < 4:
            try:
                radio.send(DRONE_MAC, outgoing.pop(0), False)
            except OSError:
                pass

        for iteration in range(4):
            try:
                peer, packet = radio.irecv(0)
            except ValueError:
                # MicroPython has already reset the desynchronized receive buffer.
                break
            if peer is None:
                break
            if peer == DRONE_MAC:
                uart.write(serial_encode(packet))
        time.sleep_ms(1)
except BaseException as error:
    # The REPL cannot reach USB while UART1 owns its pins; report the crash on UART1.
    sys.print_exception(error, uart)
    raise
