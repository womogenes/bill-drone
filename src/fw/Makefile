# Run from src/fw. FILES="main.py drivers/imu.py" uploads only those files.
.DEFAULT_GOAL := flash

ESP_BIN := $(dir $(lastword $(wildcard $(HOME)/.espressif/tools/xtensa-esp-elf/*/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc)))
export PATH := $(if $(ESP_BIN),$(ESP_BIN):)$(PATH)
ESP_PORT ?= auto
FILES ?=
REPL ?= --repl
UPLOAD_DEPS := $(if $(strip $(FILES)),$(FILES),drivers/flight_native.mpy)

.PHONY: flash upload runtime ota_runtime native flash_direct flash_runtime_direct

# Python/library changes: one connection, changed files only, then reset.
flash: $(UPLOAD_DEPS)
	uv run --with mpremote python tools/upload_usb.py --port $(ESP_PORT) $(REPL) $(FILES)

upload: REPL :=
upload: flash

# Native builds; deployed drones use the OTA layout.
runtime:
	$(MAKE) -C native runtime

ota_runtime:
	$(MAKE) -C native ota_runtime

# Direct ESP-NOW updates through the running controller bridge.
flash_direct: $(UPLOAD_DEPS)
	uv run tools/ota_direct.py files $(FILES)

flash_runtime_direct: ota_runtime
	uv run tools/ota_direct.py firmware native/build-runtime-ota/micropython.bin

native:
	$(MAKE) -C native install

drivers/flight_native.mpy: native/flight_native.c native/flight_math.h native/Makefile
	$(MAKE) -C native install
