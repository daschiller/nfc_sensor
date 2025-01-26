# SPDX-License-Identifier: GPL-3.0-only

# Copyright (c) 2022 David Schiller <david.schiller@jku.at>

# use platformio toolchain, if available
export PATH := $(HOME)/.platformio/packages/toolchain-atmelavr/bin:$(PATH)

CC = avr-gcc
OBJCOPY = avr-objcopy
FLASHER = avrdude

SRCS = src/main.c lib/i2cmaster/i2cmaster.S
INCLUDES = -Ilib/i2cmaster/
DEFS = -DF_CPU=$(F_CPU)

F_CPU = 125000UL
MCU = attiny85
CFLAGS = -Os -Wall -Wextra \
		 -flto -ffunction-sections -fdata-sections -Wl,--gc-sections
PORT = /dev/serial/by-id/usb-Silicon_Labs_CP2104_USB_to_UART_Bridge_Controller_01BE2AF7-if00-port0

ifeq ($(V), 1)
	CC += -v
	FLASHER += -v
	OBJCOPY += -v
endif

.PHONY: build clean debug flash

build: clean firmware.hex

# target-specific variable
debug: CFLAGS += -g
debug: build

firmware.elf:
	$(CC) $(CFLAGS) $(INCLUDES) -mmcu=$(MCU) $(DEFS) -o $@ $(SRCS)

firmware.hex: firmware.elf
	$(OBJCOPY) -O ihex $^ $@

flash: firmware.hex
	$(FLASHER) -P $(PORT) -p $(MCU) -c avrisp -b 19200 -U flash:w:$^:i

clean:
	rm -f firmware.elf firmware.hex
