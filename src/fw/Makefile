# Run from src/fw. The drone's ESP-IDF application in drone/.
.DEFAULT_GOAL := build
IDF_PATH ?= $(HOME)/esp/esp-idf
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build deploy usb
build:
	$(IDF) idf.py -C drone build

# Over the radio through the running laptop service: changed sectors only, then boot and confirm.
deploy: build
	uv run tools/ota_direct.py firmware drone/build/drone.bin

# Recovery only: writes the image to ota_0 and points the OTA selection back at it. The bootloader and NVS stay.
usb: build
	$(if $(PORT),,$(error Set PORT to the drone's serial device))
	$(IDF) esptool.py --chip esp32 --port $(PORT) --baud 460800 write_flash 0xd000 drone/build/ota_data_initial.bin 0x10000 drone/build/drone.bin
