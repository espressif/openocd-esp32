# SPDX-License-Identifier: GPL-2.0-or-later

# Makefile to compile flasher stub program
# Copyright (C) 2022 Espressif Systems Ltd.

# Prefix for ESP32-C6 cross compilers (can include a directory path)
CROSS ?= riscv32-esp-elf-

# Path to the esp-idf root dir
IDF_PATH ?= ../..

STUB_ARCH := riscv
STUB_CHIP_PATH := $(shell pwd)
STUB_COMMON_PATH := $(STUB_CHIP_PATH)/..
STUB_CHIP_ARCH_PATH := $(STUB_COMMON_PATH)/$(STUB_ARCH)
STUB_OBJ_DEPS := sdkconfig.h
STUB_LD_SCRIPT := stub.ld
STUB_IDF_BIN_LD_SCRIPT := stub_idf_bin.ld
STUB_STACK_SIZE := 512
STUB_CHIP := esp32c6

SRCS := $(IDF_PATH)/components/esp_hw_support/port/esp32c6/rtc_clk_init.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32c6/rtc_clk.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32c6/rtc_time.c \
		$(IDF_PATH)/components/app_trace/port/$(STUB_ARCH)/port.c

CFLAGS := -std=gnu17
CFLAGS += -Wno-dangling-pointer

INCLUDES := -I$(IDF_PATH)/components/soc/esp32c6/include -I$(IDF_PATH)/components/riscv/include \
	-I$(IDF_PATH)/components/hal/esp32c6/include \
	-I$(IDF_PATH)/components/esp32c6/include \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32c6/private_include \
	-I$(IDF_PATH)/components/esp_rom/include/esp32c6 \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32c6 \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32c6/include \
	-I$(IDF_PATH)/components/spi_flash/include \
	-I$(IDF_PATH)/components/esp_rom/esp32c6 \
	-I$(IDF_PATH)/components/esp_rom/esp32c6/include \
	-I$(IDF_PATH)/components/esp_rom/esp32c6/include/esp32c6

DEFINES :=

LDFLAGS += -T$(IDF_PATH)/components/esp_rom/esp32c6/ld/esp32c6.rom.ld -T$(IDF_PATH)/components/esp_rom/esp32c6/ld/esp32c6.rom.newlib.ld \
		-T$(IDF_PATH)/components/esp_rom/esp32c6/ld/esp32c6.rom.api.ld

include ../stub_common.mk
