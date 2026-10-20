# Native USB-to-ESP-NOW relay for the controller ESP32. Writes only the application partition.
.DEFAULT_GOAL := flash
IDF_PATH ?= $(HOME)/esp/esp-idf
PORT ?= /dev/serial/by-path/pci-0000:c1:00.3-usb-0:1:1.0-port0
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build flash
build:
	$(IDF) idf.py build

flash: build
	$(IDF) esptool.py --chip esp32 --port $(PORT) --baud 921600 write_flash 0x10000 build/relay.bin
