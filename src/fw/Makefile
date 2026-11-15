# Run from src/fw. The drone's ESP-IDF application in drone/.
.DEFAULT_GOAL := build
IDF_PATH ?= $(HOME)/esp/esp-idf
IDF = bash -c 'source $(IDF_PATH)/export.sh >/dev/null && "$$@"' idf

.PHONY: build bench bench-usb deploy usb accel level camera-test gps-test flight-test camera-fast camera-fast-wifi camera-wifi camera-deploy camera-usb
build:
	$(IDF) idf.py -C drone build

# The compute and scheduling benchmark (drone/main/bench.h, bench_flight.c): a test build in drone/build-bench with the MPC prototype and BENCH_RECORDING's sensor data for its replay (drone/main/bench_replay.h, not committed). Flash it with make bench-usb PORT=...; tools/bench.py runs it through the laptop service.
BENCH_RECORDING ?= ../sw/recordings/2026-11-15T07-02-48-960Z_659339735
bench:
	uv run tools/bench_replay.py $(BENCH_RECORDING) drone/main/bench_replay.h
	$(IDF) idf.py -C drone -B drone/build-bench -DSDKCONFIG=$(CURDIR)/drone/build-bench/sdkconfig -DBENCH=1 build

bench-usb: bench
	$(if $(PORT),,$(error Set PORT to the drone's serial device))
	$(IDF) esptool.py --chip esp32 --port $(PORT) --baud 460800 write_flash 0xd000 drone/build-bench/ota_data_initial.bin 0x10000 drone/build-bench/drone.bin

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

# The drone's camera in seconds without wiring or a network: over ESP-NOW from the FPV receiver on this laptop's USB (fw/fpv_rx, tools/camera_air_ota.py), sending only the sectors that differ. The drone powered and disarmed, the laptop service running.
camera-fast:
	uvx --from platformio==6.1.18 --with intelhex pio run -d esp32cam -e s3cam
	uv run tools/camera_air_ota.py

# The same over Wi-Fi: the camera joins the update network (the repository's .env, which this laptop must be on and which must let its clients reach each other) for one boot, takes the image, and goes back to its usual radio-only mode.
camera-fast-wifi:
	uv run tools/camera_wifi_ota.py --join

# The camera over Wi-Fi: finds it on any network this laptop is on, then builds and uploads.
camera-wifi:
	uv run tools/camera_wifi_ota.py $(if $(NAME),--name $(NAME))

# Accelerometer calibration: hold the disarmed drone still in about ten orientations.
accel:
	uv run tools/accel_cal.py

# Level reference: two still readings 180° apart on the same spot, which cancels floor tilt.
level:
	uv run tools/level.py

# Host tests of the camera link: the drone's packet parser, the radio relay's frames, and the camera's black-box format against blackbox.py.
camera-test:
	mkdir -p drone/build/host
	cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Idrone/main drone/test/camera_link_test.c -o drone/build/host/camera_link_test
	drone/build/host/camera_link_test
	cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Idrone/main drone/test/relay_link_test.c -o drone/build/host/relay_link_test
	drone/build/host/relay_link_test
	uv run esp32cam/test/blackbox_test.py

# Host test of the GPS's UBX parser and configuration request against pyubx2's frames.
gps-test:
	mkdir -p drone/build/host
	cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Idrone/main drone/test/gps_ubx_test.c -o drone/build/host/gps_ubx_test
	drone/build/host/gps_ubx_test

# Host test of the control task (drone/test/flight_rt_test.c: flight_rt.c and its modules on stubbed ESP-IDF, a crude closed-loop plant and every runtime request fuzzed): SEEDS seeds of TICKS ticks each, a digest of every output per seed. With BASE=<git ref> the same seeds also run on that commit's drone/main, and it fails unless every digest matches, for proving a change to the control task behavior-preserving.
SEEDS ?= 16
TICKS ?= 60000
FLIGHT_SOURCES = flight_rt.c flight_math.c hover.c ekf.c guard.c recover.c settings.c indi.c
flight-test:
	mkdir -p drone/build/host
	cc -std=gnu11 -O2 -Wall -Wextra -Wno-unused-parameter -Idrone/test/host -Idrone/main drone/test/flight_rt_test.c $(addprefix drone/main/,$(FLIGHT_SOURCES)) -o drone/build/host/flight_rt_test -lm
	$(if $(BASE),rm -rf drone/build/host/base && mkdir -p drone/build/host/base && git archive $(BASE) drone/main | tar -x -C drone/build/host/base && for f in $(FLIGHT_SOURCES); do [ -e drone/build/host/base/drone/main/$$f ] || : > drone/build/host/base/drone/main/$$f; done)
	$(if $(BASE),cc -std=gnu11 -O2 -w -Idrone/test/host -Idrone/build/host/base/drone/main drone/test/flight_rt_test.c $(addprefix drone/build/host/base/drone/main/,$(FLIGHT_SOURCES)) -o drone/build/host/flight_rt_base -lm)
	for seed in $$(seq 1 $(SEEDS)); do \
		new=$$(drone/build/host/flight_rt_test $$seed $(TICKS) | tail -1); echo "seed $$seed: $$new"; \
		$(if $(BASE),old=$$(drone/build/host/flight_rt_base $$seed $(TICKS) | tail -1); [ "$$old" = "$$new" ] || { echo "seed $$seed differs from $(BASE): $$old"; exit 1; };) \
	done
