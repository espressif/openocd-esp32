# SPDX-License-Identifier: GPL-2.0-or-later

# Makefile to compile flasher stub program
# Copyright (C) 2026 Espressif Systems Ltd.

# Prefix for ESP32-H21 cross compilers (can include a directory path)
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
STUB_STACK_SIZE := 512
STUB_CHIP := esp32h21

SRCS := $(IDF_PATH)/components/esp_hw_support/port/esp32h21/rtc_clk_init.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32h21/rtc_clk.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32h21/rtc_time.c

CFLAGS := -std=gnu17
CFLAGS += -Wno-dangling-pointer

INCLUDES := -I$(IDF_PATH)/components/soc/esp32h21/include -I$(IDF_PATH)/components/riscv/include \
	-I$(IDF_PATH)/components/hal/esp32h21/include \
	-I$(IDF_PATH)/components/esp32h21/include \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32h21/private_include \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32h21/include \
	-I$(IDF_PATH)/components/esp_rom/include/esp32h21 \
	-I$(IDF_PATH)/components/esp_hw_support/port/esp32h21 \
	-I$(IDF_PATH)/components/spi_flash/include \
	-I$(IDF_PATH)/components/esp_rom/esp32h21 \
	-I$(IDF_PATH)/components/esp_rom/esp32h21/include \
	-I$(IDF_PATH)/components/esp_rom/esp32h21/include/esp32h21 \
	-I$(IDF_PATH)/components/esp_hal_gpio/esp32h21/include \
	-I$(IDF_PATH)/components/esp_hal_timg/esp32h21/include \
	-I$(IDF_PATH)/components/esp_hal_pmu/esp32h21/include \
	-I$(IDF_PATH)/components/esp_hal_rtc_timer/esp32h21/include \
	-I$(IDF_PATH)/components/esp_hal_clock/esp32h21/include \
	-I$(IDF_PATH)/components/soc/esp32h21/register

DEFINES :=

LDFLAGS += -T$(IDF_PATH)/components/esp_rom/esp32h21/ld/esp32h21.rom.ld -T$(IDF_PATH)/components/esp_rom/esp32h21/ld/esp32h21.rom.newlib.ld \
		-T$(IDF_PATH)/components/esp_rom/esp32h21/ld/esp32h21.rom.api.ld

include ../stub_common.mk
