# The FPV receiver ESP32 (main/fpv_rx.c). Writes only the application partition after the first full flash (make full).
.DEFAULT_GOAL := flash
IDF_PATH ?= $(HOME)/esp/esp-idf
PORT ?= /dev/serial/by-path/pci-0000:c1:00.3-usb-0:2.2:1.0-port0
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build flash full
build:
	$(IDF) idf.py build

flash: build
	$(IDF) esptool.py --chip esp32 --port $(PORT) --baud 921600 write_flash 0x10000 build/fpv_rx.bin

full: build
	$(IDF) idf.py -p $(PORT) -b 921600 flash
