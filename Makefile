# HDC Sense & Control Node — build & deploy
#
# Usage:  make <target>            (defaults to the esp32-s3-eth env)
#         make upload ENV=rack-a01 (target a specific fleet env)
#         make PORT=/dev/cu.usbmodemXXXX upload
#
# PlatformIO isn't on PATH by default; override PIO= if yours differs.

PIO  ?= $(HOME)/.platformio/penv/bin/pio
ENV  ?= esp32-s3-eth
PORT ?=
BAUD ?= 115200

# Optional --upload-port / --monitor-port if PORT is set.
PORTFLAG := $(if $(PORT),--upload-port $(PORT),)
MONFLAG  := $(if $(PORT),--port $(PORT),)

# `make` on its own compiles the firmware.
.DEFAULT_GOAL := build

help: ## Show available commands
	@echo "HDC Sense & Control Node — make targets (ENV=$(ENV)):"
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
	  | awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-11s\033[0m %s\n",$$1,$$2}'

BNFILE := .build_number

bump: ## Increment the build number
	@n=$$(cat $(BNFILE) 2>/dev/null || echo 0); n=$$((n+1)); printf "%s" "$$n" > $(BNFILE); echo "  build #$$n"

build: bump ## Compile the firmware (default)
	$(PIO) run -e $(ENV)

deploy: bump ## Compile + flash the ESP over USB
	$(PIO) run -e $(ENV) -t upload $(PORTFLAG)

upload: deploy ## Alias for deploy
flash: deploy  ## Alias for deploy

monitor: ## Open the serial monitor
	$(PIO) device monitor -b $(BAUD) $(MONFLAG)

dev: ## Flash, then open the serial monitor
	$(PIO) run -e $(ENV) -t upload $(PORTFLAG) && $(PIO) device monitor -b $(BAUD) $(MONFLAG)

erase: ## Erase flash + NVS (return to factory/first-boot state)
	$(PIO) run -e $(ENV) -t erase $(PORTFLAG)

provision: ## Erase + flash + monitor — full fresh-unit provisioning
	$(PIO) run -e $(ENV) -t erase $(PORTFLAG) \
	  && $(PIO) run -e $(ENV) -t upload $(PORTFLAG) \
	  && $(PIO) device monitor -b $(BAUD) $(MONFLAG)

size: ## Report flash/RAM usage
	$(PIO) run -e $(ENV) -t size

clean: ## Remove build artifacts for this env
	$(PIO) run -e $(ENV) -t clean

distclean: ## Remove the entire .pio build directory
	rm -rf .pio

devices: ## List connected serial devices
	$(PIO) device list

.PHONY: help bump build deploy upload flash monitor dev erase provision size clean distclean devices
