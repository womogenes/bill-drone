# Much better than commands in readmes

flash_libs:
	mpremote fs cp lib/microdot/*.mpy :/lib/microdot/ + reset repl

flash:
	mpremote fs cp -r config.py main.py drivers : + reset repl

ESP_IP ?= 192.168.86.32

flash_web: SHELL := /bin/bash
flash_web:
	@read -rsp 'WebREPL password: ' password; echo; \
	for file in config.py main.py drivers/*.py; do \
		uv run lib/webrepl/webrepl_cli.py -p "$$password" "$$file" "$(ESP_IP):/$$file" || exit $$?; \
	done; \
	uv run lib/webrepl/webrepl_cli.py -p "$$password" --reset --repl "$(ESP_IP)"

repl_web:
	uv run lib/webrepl/webrepl_cli.py $(ESP_IP)
