# The thrust stand's ESP32 (main/stand.c): one ESC and up to two HX711 load cells, driven from the laptop by sw/stand.py over USB or Wi-Fi.
.DEFAULT_GOAL := flash
IDF_PATH ?= $(HOME)/esp/esp-idf
PORT ?= /dev/ttyUSB1
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build flash ota
build:
	$(IDF) idf.py build

flash: build
	$(IDF) idf.py -p $(PORT) -b 460800 flash

# Over Wi-Fi to the stand found by its broadcast (main/stand.c, port 3335).
ota: build
	cd ../.. && uv run -m sw.stand flash
