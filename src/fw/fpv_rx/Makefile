# The FPV receiver (main/fpv_rx.c): an ESP32 (build, flash, full; flash writes only the application partition after the first full flash) or an ESP32-S3 on its own USB (s3, s3-flash PORT=/dev/ttyACM0; sdkconfig.defaults.esp32s3).
.DEFAULT_GOAL := flash
IDF_PATH ?= $(HOME)/esp/esp-idf
PORT ?= /dev/serial/by-path/pci-0000:c1:00.3-usb-0:2.2:1.0-port0
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build flash full s3 s3-flash
build:
	$(IDF) idf.py build

flash: build
	$(IDF) esptool.py --chip esp32 --port $(PORT) --baud 921600 write_flash 0x10000 build/fpv_rx.bin

full: build
	$(IDF) idf.py -p $(PORT) -b 921600 flash

S3 = $(IDF) idf.py -B build-s3 -D IDF_TARGET=esp32s3 -D SDKCONFIG=sdkconfig.s3
s3:
	$(S3) build

s3-flash: s3
	$(IDF) idf.py -B build-s3 -D SDKCONFIG=sdkconfig.s3 -p $(PORT) flash
