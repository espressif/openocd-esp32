# SPDX-License-Identifier: GPL-2.0-or-later

# Makefile to compile flasher stub program
# Copyright (C) 2019 Espressif Systems Ltd.

# Prefix for ESP32 cross compilers (can include a directory path)
CROSS ?= xtensa-esp32-elf-

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
STUB_CHIP := esp32

SRCS := $(IDF_PATH)/components/app_trace/port/$(STUB_ARCH)/port.c \
	$(IDF_PATH)/components/esp_hw_support/port/esp32/rtc_clk.c \
	$(IDF_PATH)/components/esp_hw_support/port/esp32/rtc_clk_init.c \
	$(IDF_PATH)/components/esp_hw_support/port/esp32/rtc_time.c \
	$(IDF_PATH)/components/xtensa/eri.c \
	$(STUB_CHIP_PATH)/stub_sha_chip.c \
	$(STUB_CHIP_PATH)/stub_spiflash_rom_patch.c

CFLAGS := -std=gnu99 -mlongcalls -mtext-section-literals

INCLUDES := -I$(IDF_PATH)/components/esp32/include \
	-I$(IDF_PATH)/components/soc/esp32/include \
	-I$(IDF_PATH)/components/efuse/esp32/include \
	-I$(IDF_PATH)/components/esp_rom/include/esp32 \
	-I$(IDF_PATH)/components/xtensa/esp32/include \
	-I$(IDF_PATH)/components/hal/esp32/include \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32 \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32/private_include

DEFINES :=

LDFLAGS := -L$(IDF_PATH)/components/esp32/ld -T$(IDF_PATH)/components/esp_rom/esp32/ld/esp32.rom.ld \
	-T$(IDF_PATH)/components/esp_rom/esp32/ld/esp32.rom.newlib-funcs.ld \
	-T$(IDF_PATH)/components/esp_rom/esp32/ld/esp32.rom.libgcc.ld -T$(IDF_PATH)/components/esp_rom/esp32/ld/esp32.rom.api.ld \
	-T$(IDF_PATH)/components/esp_rom/esp32/ld/esp32.rom.redefined.ld -T$(IDF_PATH)/components/esp_rom/esp32/ld/esp32.rom.spiflash.ld

include ../stub_common.mk
