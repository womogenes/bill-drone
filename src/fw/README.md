# Firmware

The gist behind this is that the MCU executes Python instead of machine instructions so we can iterate on firmware much faster.

## MicroPython

To install MicroPython:

```sh
# Run this from src/
uv tool install mpremote
mpremote --help

# Install micropython
# Assumes /dev/ttyUSB0 is the ESP32's port
uvx --from esptool esptool chip-id
uvx --from esptool esptool --port /dev/ttyUSB0 erase-flash

# Download firmware (v1.29, from Aug 2026)
wget -P https://micropython.org/resources/firmware/ESP32_GENERIC-20260824-v1.29.0.bin -o bin/
uvx --from esptool esptool --port /dev/ttyUSB0 --baud 460800 write-flash 0x1000 bin/ESP32_GENERIC-20260824-v1.29.0.bin

# Install deps
# lib/ should be copied manually per below
mpremote mip install requests

# Connect:
uvx mpremote repl
```

## Dependencies

Microdot is a small web framework. We take it from
https://github.com/miguelgrinberg/microdot/tree/7742db9ff9f49635de3387145fafe56bf2377a97/src/microdot
and store its original sources in `src/lib/microdot/`. The firmware's
`lib/microdot/` contains precompiled `.mpy` files for HTTP and WebSocket support.
Compiling the sources on the ESP32 exhausts its native networking heap
(`OSError: -203`), so compile them on the laptop instead.

To rebuild with the MicroPython 1.29 compiler, run from `src/`:

```sh
for file in lib/microdot/*.py; do
    uvx --from mpy-cross==1.29.0.post2 mpy-cross "$file" -o "fw/lib/microdot/$(basename "${file%.py}").mpy"
done
```

When replacing an existing source installation, remove these once; `.py` files
take precedence over `.mpy` files (already done on our ESP32):

```sh
mpremote fs rm :/lib/microdot/__init__.py :/lib/microdot/microdot.py :/lib/microdot/websocket.py :/lib/microdot/helpers.py
```

If the board has the old standalone `/lib/microdot.py`, remove it once:

```sh
mpremote fs rm :/lib/microdot.py
```

## Code structure

```sh
cd src/fw

# Update libs
mpremote fs cp -r lib : + reset repl

# Upload stuff + reset
mpremote fs cp -r main.py drivers : + reset repl
```

This copies `main.py` and `drivers/` to `:` (remote) and resets the processor.

To run without resetting, run

```sh
mpremote resume mount . run test.py repl
```
