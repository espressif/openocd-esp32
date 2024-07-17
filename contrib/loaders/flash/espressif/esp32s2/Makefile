# SPDX-License-Identifier: GPL-2.0-or-later

# Makefile to compile flasher stub program
# Copyright (C) 2021 Espressif Systems Ltd.

# Prefix for ESP32-S2 cross compilers (can include a directory path)
CROSS ?= xtensa-esp32s2-elf-

# Path to the esp-idf root dir
IDF_PATH ?= ../..

STUB_ARCH := xtensa
STUB_CHIP_PATH := $(shell pwd)
STUB_COMMON_PATH := $(STUB_CHIP_PATH)/..
STUB_CHIP_ARCH_PATH := $(STUB_COMMON_PATH)/$(STUB_ARCH)
STUB_OBJ_DEPS := sdkconfig.h
STUB_LD_SCRIPT := stub.ld
STUB_IDF_BIN_LD_SCRIPT := stub.ld
STUB_STACK_SIZE := 512
STUB_CHIP := esp32s2

SRCS := $(IDF_PATH)/components/app_trace/port/$(STUB_ARCH)/port.c \
	$(IDF_PATH)/components/xtensa/eri.c \
	$(IDF_PATH)/components/esp_hw_support/port/esp32s2/rtc_clk.c \
	$(IDF_PATH)/components/esp_hw_support/port/esp32s2/rtc_clk_init.c \
	$(IDF_PATH)/components/esp_hw_support/port/esp32s2/rtc_time.c \
	$(IDF_PATH)/components/esp_rom/patches/esp_rom_regi2c.c

CFLAGS := -std=gnu99 -mlongcalls -mtext-section-literals

INCLUDES := -I$(IDF_PATH)/components/esp32s2/include -I$(IDF_PATH)/components/soc/esp32s2/include \
	-I$(IDF_PATH)/components/esp_rom/include/esp32s2 -I$(IDF_PATH)/components/xtensa/esp32s2/include \
	-I$(IDF_PATH)/components/hal/esp32s2/include -I$(IDF_PATH)/components/esp_rom/esp32s2 \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32s2/private_include \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32s2 \

DEFINES :=

LDFLAGS := -L$(IDF_PATH)/components/esp32s2/ld -T$(IDF_PATH)/components/esp_rom/esp32s2/ld/esp32s2.rom.ld \
	-T$(IDF_PATH)/components/esp_rom/esp32s2/ld/esp32s2.rom.newlib-funcs.ld \
	-T$(IDF_PATH)/components/esp_rom/esp32s2/ld/esp32s2.rom.spiflash.ld -T$(IDF_PATH)/components/esp_rom/esp32s2/ld/esp32s2.rom.api.ld \
	-T$(IDF_PATH)/components/esp_rom/esp32s2/ld/esp32s2.rom.libgcc.ld

include ../stub_common.mk