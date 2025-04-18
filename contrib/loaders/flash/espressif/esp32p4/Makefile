# SPDX-License-Identifier: GPL-2.0-or-later

# Makefile to compile flasher stub program
# Copyright (C) 2024 Espressif Systems Ltd.

# Prefix for ESP32-P4 cross compilers (can include a directory path)
CROSS ?= riscv32-esp-elf-

# Path to the esp-idf root dir
IDF_PATH ?= ../..

STUB_ARCH := riscv
STUB_CHIP_PATH := $(shell pwd)
STUB_COMMON_PATH := $(STUB_CHIP_PATH)/..
STUB_CHIP_ARCH_PATH := $(STUB_COMMON_PATH)/$(STUB_ARCH)
STUB_OBJ_DEPS := sdkconfig.h
STUB_LD_SCRIPT := stub.ld
STUB_IDF_BIN_LD_SCRIPT := stub.ld
STUB_STACK_SIZE := 1024
STUB_CHIP := esp32p4

SRCS := $(IDF_PATH)/components/esp_hw_support/port/esp32p4/rtc_clk_init.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32p4/rtc_clk.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32p4/rtc_time.c \
		$(IDF_PATH)/components/app_trace/port/$(STUB_ARCH)/port.c

CFLAGS := -std=gnu17
CFLAGS += -Wno-dangling-pointer

INCLUDES := -I$(IDF_PATH)/components/soc/esp32p4/include -I$(IDF_PATH)/components/riscv/include \
	-I$(IDF_PATH)/components/hal/esp32p4/include \
	-I$(IDF_PATH)/components/esp32p4/include \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32p4/private_include \
	-I$(IDF_PATH)/components/esp_rom/include/esp32p4 \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32p4 \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32p4/include \
	-I$(IDF_PATH)/components/spi_flash/include \
	-I$(IDF_PATH)/components/esp_rom/esp32p4 \
	-I$(IDF_PATH)/components/esp_rom/esp32p4/include \
	-I$(IDF_PATH)/components/esp_rom/esp32p4/include/esp32p4

DEFINES :=

LDFLAGS += -T$(IDF_PATH)/components/esp_rom/esp32p4/ld/esp32p4.rom.ld -T$(IDF_PATH)/components/esp_rom/esp32p4/ld/esp32p4.rom.newlib.ld \
		-T$(IDF_PATH)/components/esp_rom/esp32p4/ld/esp32p4.rom.api.ld

include ../stub_common.mk
