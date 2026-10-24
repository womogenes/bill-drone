# Run from src/fw. The drone's ESP-IDF application in drone/.
.DEFAULT_GOAL := build
IDF_PATH ?= $(HOME)/esp/esp-idf
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build deploy usb accel level camera-test
build:
	$(IDF) idf.py -C drone build

# Over the radio through the running laptop service: changed sectors only, then boot and confirm.
deploy: build
	uv run tools/ota_direct.py firmware drone/build/drone.bin

# Recovery only: writes the image to ota_0 and points the OTA selection back at it. The bootloader and NVS stay.
usb: build
	$(if $(PORT),,$(error Set PORT to the drone's serial device))
	$(IDF) esptool.py --chip esp32 --port $(PORT) --baud 460800 write_flash 0xd000 drone/build/ota_data_initial.bin 0x10000 drone/build/drone.bin

# Accelerometer calibration: hold the disarmed drone still in about ten orientations.
accel:
	uv run tools/accel_cal.py

# Level reference: two still readings 180° apart on the same spot, which cancels floor tilt.
level:
	uv run tools/level.py

# Host tests of the camera link: the drone's packet parser, and the camera's black-box format against blackbox.py.
camera-test:
	mkdir -p drone/build/host
	cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Idrone/main drone/test/camera_link_test.c -o drone/build/host/camera_link_test
	drone/build/host/camera_link_test
	uv run esp32cam/test/blackbox_test.py
