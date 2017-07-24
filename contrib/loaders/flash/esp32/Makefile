# Makefile to compile the flasher stub program
#
# Note that YOU DO NOT NEED TO COMPILE THIS IN ORDER TO JUST USE

# See the comments in the top of the Makefile for parameters that
# you probably want to override.
#
# Copyright (c) 2017 Espressif Systems
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

WRAPPER_SRC = stub_wrapper.S

BUILD_DIR = build

STUB_ELF = $(BUILD_DIR)/$(STUB).elf
STUB_OBJ = $(BUILD_DIR)/$(STUB).o
STUB_CODE_SECT = $(STUB)_code.inc
STUB_DATA_SECT = $(STUB)_data.inc
STUB_WRAPPER = $(STUB)_wrapper.inc
STUB_IMAGE_HDR = $(STUB)_image.h

.PHONY: all clean

all: $(STUB_ELF) $(STUB_WRAPPER) $(STUB_IMAGE_HDR) $(STUB_CODE_SECT) $(STUB_DATA_SECT)

$(BUILD_DIR):
	$(Q) mkdir $@

CFLAGS = -std=c99 -Wall -Werror -Os \
         -mtext-section-literals -mlongcalls -nostdlib -fno-builtin -flto \
         -Wl,-static -g -ffunction-sections -Wl,--gc-sections

INCLUDES += -I. -I$(IDF_PATH)/components/esp32/include -I$(IDF_PATH)/components/soc/esp32/include \
		-I$(IDF_PATH)/components/app_trace/include -I$(IDF_PATH)/components/xtensa-debug-module/include \
		-I$(IDF_PATH)/components/freertos/include -I$(IDF_PATH)/components/log/include \
		-L$(IDF_PATH)/components/esp32/ld

DEFINES = -Dasm=__asm__

CFLAGS += $(INCLUDES) $(DEFINES)

$(STUB_ELF): $(SRCS) stub.ld $(BUILD_DIR)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -DSTUB_IMAGE=1 -Tstub.ld -Wl,-Map=$(@:.elf=.map) -o $@ $(filter %.c, $^)

$(STUB_OBJ): $(SRCS) sdkconfig.h
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -c $(filter %.c, $^) -o $@

$(STUB_CODE_SECT): $(STUB_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)objcopy -O binary -j.text $^ $(BUILD_DIR)/$(STUB)_code.bin
	$(Q) cat $(BUILD_DIR)/$(STUB)_code.bin | xxd -i > $@

$(STUB_DATA_SECT): $(STUB_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)objcopy -O binary -j.data $^ $(BUILD_DIR)/$(STUB)_data.bin
	$(Q) cat $(BUILD_DIR)/$(STUB)_data.bin | xxd -i > $@

$(STUB_WRAPPER): $(WRAPPER_SRC)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc -c $^ -o $(BUILD_DIR)/$(STUB)_wrapper.o
	$(Q) $(CROSS)objcopy -O binary $(BUILD_DIR)/$(STUB)_wrapper.o $(BUILD_DIR)/$(STUB)_wrapper.bin
	$(Q) cat $(BUILD_DIR)/$(STUB)_wrapper.bin | xxd -i > $@

$(STUB_IMAGE_HDR): $(STUB_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) @echo -n "#define ESP32_STUB_BSS_SIZE 0x" > $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -S $^ | fgrep .bss | awk '{print $$7}' >> $(STUB_IMAGE_HDR)
	$(Q) @echo -n "#define ESP32_STUB_ENTRY_ADDR 0x" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep stub_main | awk '{print $$2}' >> $(STUB_IMAGE_HDR)

clean:
	$(Q) rm -rf $(BUILD_DIR) $(STUB_CODE_SECT) $(STUB_DATA_SECT) $(STUB_WRAPPER) $(STUB_IMAGE_HDR)
