# Makefile to compile the flasher stub program
#
# Note that YOU DO NOT NEED TO COMPILE THIS IN ORDER TO JUST USE
# esptool.py - a precompiled version is embedded in esptool.py,
# so if you don't want to modify the stub code then you are good to go.
#
# See the comments in the top of the Makefile for parameters that
# you probably want to override.
#
# Copyright (c) 2016 Cesanta Software Limited & Angus Gratton
# All rights reserved
#
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
# Street, Fifth Floor, Boston, MA 02110-1301 USA.

# Adapted from Cesanta's original Makefile at
# https://github.com/cesanta/fnc/tree/master/common/platforms/esp8266/stubs

# Override these variables on the command line
# or set them in a local.mk

#-include local.mk

# Prefix for ESP32 cross compilers (can include a directory path)
CROSS ?= xtensa-esp32-elf-

# Path to the esp-idf root dir
IDF_PATH ?= ../..

# Pass V=1 to see the commands being executed by make
ifneq ("$(V)","1")
Q = @
endif

STUB = stub_flasher
SRCS = stub_flasher.c \
	$(IDF_PATH)/components/spi_flash/spi_flash_rom_patch.c \
	$(IDF_PATH)/components/app_trace/app_trace.c \
	$(IDF_PATH)/components/app_trace/app_trace_util.c \
	$(IDF_PATH)/components/xtensa-debug-module/eri.c \
	$(IDF_PATH)/components/soc/esp32/rtc_clk.c \
	$(IDF_PATH)/components/soc/esp32/rtc_time.c

BUILD_DIR = build

STUB_ELF = $(BUILD_DIR)/$(STUB).elf
STUB_OBJ = $(BUILD_DIR)/$(STUB).o

.PHONY: all clean

all: $(STUB_ELF)

$(BUILD_DIR):
	$(Q) mkdir $@

#-Werror
CFLAGS = -std=c99 -Wall -Os \
         -mtext-section-literals -mlongcalls -nostdlib -fno-builtin -flto \
         -Wl,-static -g -ffunction-sections -Wl,--gc-sections

INCLUDES += -I. -I$(IDF_PATH)/components/esp32/include -I$(IDF_PATH)/components/soc/esp32/include \
		-I$(IDF_PATH)/components/app_trace/include -I$(IDF_PATH)/components/xtensa-debug-module/include \
		-I$(IDF_PATH)/components/freertos/include -I$(IDF_PATH)/components/log/include \
		-L$(IDF_PATH)/components/esp32/ld

DEFINES = -Dasm=__asm__

CFLAGS += $(INCLUDES) $(DEFINES)

$(STUB_ELF): $(SRCS) sdkconfig.h stub.ld $(BUILD_DIR)
	@echo "  CC(32)   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -Tstub.ld -Wl,-Map=$(@:.elf=.map) -o $@ $(filter %.c, $^)

$(STUB_OBJ): $(SRCS) sdkconfig.h $(BUILD_DIR)
	@echo "  CC(32)   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -c $(filter %.c, $^) -o $@

clean:
	$(Q) rm -rf $(BUILD_DIR)
