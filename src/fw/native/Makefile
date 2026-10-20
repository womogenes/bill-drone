.DEFAULT_GOAL := install
MPY_DIR ?= $(CURDIR)/.micropython

ifeq ($(filter host runtime ota_runtime,$(MAKECMDGOALS)),)
$(MPY_DIR)/py/dynruntime.mk:
	git clone --depth 1 --branch v1.29.0 https://github.com/micropython/micropython.git $(MPY_DIR)

MOD = flight_native
SRC = flight_native.c
ARCH = xtensawin
LINK_RUNTIME = 1
include $(MPY_DIR)/py/dynruntime.mk
PYTHON = uv run --quiet --project ../.. python
$(BUILD)/flight_native.o: flight_math.h
endif

.PHONY: host install runtime ota_runtime
runtime:
	bash runtime/build.sh

ota_runtime:
	bash runtime/build.sh ota

host: flight_native.so

flight_native.so: flight_native.c flight_math.h
	cc -O2 -fPIC -shared $< -o $@.tmp -lm
	mv $@.tmp $@

install: all
	cp flight_native.mpy ../drivers/flight_native.mpy
