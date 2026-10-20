# Run from src/fw. FILES="main.py drivers/imu.py" uploads only those files.
.DEFAULT_GOAL := flash

ESP_BIN := $(dir $(lastword $(wildcard $(HOME)/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc)))
export PATH := $(if $(ESP_BIN),$(ESP_BIN):)$(PATH)
ESP_IP ?= 192.168.86.32
ESP_PORT ?= /dev/ttyUSB0
ESP_BAUD ?= 115200
FILES ?=
REPL ?= --repl
MICRODOT := $(patsubst %.py,%.mpy,$(wildcard lib/microdot/*.py))
UPLOAD_DEPS := $(if $(strip $(FILES)),$(FILES),drivers/flight_native.mpy $(MICRODOT))

.PHONY: flash upload flash_web upload_web flash_libs flash_runtime flash_runtime_web runtime ota_runtime native repl_web migrate_ota flash_direct flash_runtime_direct

# Python/library changes: one connection, changed files only, then reset.
flash: $(UPLOAD_DEPS)
	uv run --with mpremote python tools/upload_usb.py --port $(ESP_PORT) $(REPL) $(FILES)

upload: REPL :=
upload: flash

flash_web: $(UPLOAD_DEPS)
	uv run python tools/flash_web.py $(ESP_IP) $(REPL) $(FILES)

upload_web: REPL :=
upload_web: flash_web

flash_libs: $(MICRODOT)
	uv run --with mpremote python tools/upload_usb.py --port $(ESP_PORT) $(REPL) $(MICRODOT)

# C changes: rebuild and replace the application partition, preserving files.
runtime:
	$(MAKE) -C native runtime

ota_runtime:
	$(MAKE) -C native ota_runtime

flash_runtime: runtime
	uv run --with esptool==5.6.0 python tools/flash_native.py --port $(ESP_PORT) --baud $(ESP_BAUD) native/build-runtime/micropython.bin
	$(MAKE) flash

# Requires the reviewed one-time OTA partition migration over USB.
flash_runtime_web: ota_runtime
	uv run python tools/ota.py $(ESP_IP) native/build-runtime-ota/micropython.bin $(REPL)

# One-time move to the OTA layout over USB; DRONE_PORT must name the drone, not the controller.
migrate_ota: ota_runtime
	uv run --with esptool==5.6.0 --with mpremote python tools/migrate_ota.py --port $(DRONE_PORT)

# Direct ESP-NOW updates through the running controller bridge.
flash_direct: $(UPLOAD_DEPS)
	uv run tools/ota_direct.py files $(FILES)

flash_runtime_direct: ota_runtime
	uv run tools/ota_direct.py firmware native/build-runtime-ota/micropython.bin

repl_web:
	uv run lib/webrepl/webrepl_cli.py $(ESP_IP)

native:
	$(MAKE) -C native install

drivers/flight_native.mpy: native/flight_native.c native/flight_math.h native/Makefile
	$(MAKE) -C native install

lib/microdot/%.mpy: lib/microdot/%.py
	uvx --from mpy-cross==1.29.0.post2 mpy-cross $< -o $@
