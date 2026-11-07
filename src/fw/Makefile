# Run from src/fw. The drone's ESP-IDF application in drone/.
.DEFAULT_GOAL := build
IDF_PATH ?= $(HOME)/esp/esp-idf
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build deploy usb accel level camera-test gps-test camera-wifi camera-deploy camera-usb
build:
	$(IDF) idf.py -C drone build

# Over the radio through the running laptop service: changed sectors only, then boot and confirm.
deploy: build
	uv run tools/ota_direct.py firmware drone/build/drone.bin

# Recovery only: writes the image to ota_0 and points the OTA selection back at it. The bootloader and NVS stay.
usb: build
	$(if $(PORT),,$(error Set PORT to the drone's serial device))
	$(IDF) esptool.py --chip esp32 --port $(PORT) --baud 460800 write_flash 0xd000 drone/build/ota_data_initial.bin 0x10000 drone/build/drone.bin

# The camera through the drone's radio link and its wire to the camera: no Wi-Fi needed, the drone disarmed and running firmware with camera updates.
camera-deploy:
	uvx --from platformio==6.1.18 --with intelhex pio run -d esp32cam -e s3cam
	uv run tools/ota_direct.py camera esp32cam/.pio/build/s3cam/firmware.bin

# The S3 camera over its own USB-C, the first time or for recovery (the drone's 6 V off while USB is in).
camera-usb:
	$(if $(PORT),,$(error Set PORT to the camera's USB device, e.g. /dev/ttyACM0))
	uvx --from platformio==6.1.18 --with intelhex pio run -d esp32cam -e s3cam -t upload --upload-port $(PORT)

# The camera over Wi-Fi: finds it on any network this laptop is on, then builds and uploads.
camera-wifi:
	uv run tools/camera_wifi_ota.py $(if $(NAME),--name $(NAME))

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

# Host test of the GPS's UBX parser and configuration request against pyubx2's frames.
gps-test:
	mkdir -p drone/build/host
	cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Idrone/main drone/test/gps_ubx_test.c -o drone/build/host/gps_ubx_test
	drone/build/host/gps_ubx_test
