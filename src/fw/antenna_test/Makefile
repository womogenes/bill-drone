# Antenna test for an ESP32 board (main/antenna_test.c). Put the board in download mode (IO0 to GND, then RST) before make flash.
.DEFAULT_GOAL := flash
IDF_PATH ?= $(HOME)/esp/esp-idf
PORT ?= /dev/ttyUSB0
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build flash
build:
	$(IDF) idf.py build

flash: build
	$(IDF) idf.py -p $(PORT) -b 460800 flash
