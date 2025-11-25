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
STUB_STACK_SIZE := 1024
STUB_CHIP := esp32p4

HW_REVISIONS := 1 3
BUILD_DIRS := $(addprefix build_v,$(HW_REVISIONS))
INC_DIRS   := $(addprefix inc_v,$(HW_REVISIONS))

ifeq ($(HW_REV),)

.PHONY: all $(HW_REVISIONS)

all: $(HW_REVISIONS)

clean:
	@echo "Cleaning all build and inc directories"
	@rm -rf $(BUILD_DIRS) $(INC_DIRS)

$(HW_REVISIONS):
	$(MAKE) HW_REV=$@ $(MAKECMDGOALS)

else

ifeq ($(HW_REV),1)
ESP32P4_REG_VER := hw_ver1
ESP32P4_ROM_LD := esp32p4.rom.ld
ESP32P4_ROM_NEWLIB := esp32p4.rom.newlib.ld
STUB_LD_SCRIPT := stub_v1.ld
STUB_IDF_BIN_LD_SCRIPT := stub_v1.ld
else ifeq ($(HW_REV),3)
ESP32P4_REG_VER := hw_ver3
ESP32P4_ROM_LD := esp32p4.rom.eco5.ld
ESP32P4_ROM_NEWLIB := esp32p4.rom.eco5.newlib.ld
STUB_LD_SCRIPT := stub_v3.ld
STUB_IDF_BIN_LD_SCRIPT := stub_v3.ld
DEFINES += -DCONFIG_ESP_REV_MIN_FULL=300
else
$(error Unsupported HW_REV=$(HW_REV). Expected 1 or 3)
endif

BUILD_DIR ?= build_v$(HW_REV)
INC_DIR ?= inc_v$(HW_REV)

SRCS := $(IDF_PATH)/components/esp_hw_support/port/esp32p4/rtc_clk_init.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32p4/rtc_clk.c \
		$(IDF_PATH)/components/esp_hw_support/port/esp32p4/rtc_time.c

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
	-I$(IDF_PATH)/components/soc/esp32p4/register/$(ESP32P4_REG_VER) \
	-I$(IDF_PATH)/components/esp_libc/platform_include \
	-I$(IDF_PATH)/components/esp_rom/esp32p4/include/esp32p4 \
	-I$(IDF_PATH)/components/esp_hal_timg/esp32p4/include/ \
	-I$(IDF_PATH)/components/esp_hal_gpio/include/ \
	-I$(IDF_PATH)/components/esp_hal_gpio/esp32p4/include/


LDFLAGS += -T$(IDF_PATH)/components/esp_rom/esp32p4/ld/$(ESP32P4_ROM_LD) \
		-T$(IDF_PATH)/components/esp_rom/esp32p4/ld/$(ESP32P4_ROM_NEWLIB) \
		-T$(IDF_PATH)/components/esp_rom/esp32p4/ld/esp32p4.rom.api.ld

include ../stub_common.mk

endif
