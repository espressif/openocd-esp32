# Common Makefile rules to compile the flasher stub program
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

# Pass V=1 to see the commands being executed by make
ifneq ("$(V)","1")
Q = @
endif

STUB = stub_flasher
SRCS += $(STUB_COMMON_PATH)/stub_flasher.c \
	$(STUB_CHIP_PATH)/stub_flasher_chip.c \
	$(IDF_PATH)/components/spi_flash/spi_flash_rom_patch.c \
	$(IDF_PATH)/components/app_trace/app_trace.c \
	$(IDF_PATH)/components/app_trace/app_trace_util.c

BUILD_DIR = build

STUB_ELF = $(BUILD_DIR)/$(STUB).elf
STUB_OBJ = $(BUILD_DIR)/$(STUB).o
STUB_CODE_SECT = $(STUB)_code.inc
STUB_DATA_SECT = $(STUB)_data.inc
STUB_IMAGE_HDR = $(STUB)_image.h

.PHONY: all clean

all: $(STUB_ELF) $(STUB_IMAGE_HDR) $(STUB_CODE_SECT) $(STUB_DATA_SECT)

$(BUILD_DIR):
	$(Q) mkdir $@

CFLAGS += -std=c99 -Wall -Werror -Os \
         -mtext-section-literals -mlongcalls -nostdlib -fno-builtin -flto \
         -Wl,-static -g -ffunction-sections -Wl,--gc-sections

INCLUDES += -I. -I$(STUB_COMMON_PATH) -I$(STUB_CHIP_PATH) -I$(IDF_PATH)/components/soc/include \
		-I$(IDF_PATH)/components/app_trace/include -I$(IDF_PATH)/components/xtensa-debug-module/include -I$(IDF_PATH)/components/driver/include \
		-I$(IDF_PATH)/components/freertos/include -I$(IDF_PATH)/components/log/include -I$(IDF_PATH)/components/heap/include \
		-I$(IDF_PATH)/components/bootloader_support/include

DEFINES += -Dasm=__asm__

CFLAGS += $(INCLUDES) $(DEFINES)

LDFLAGS += -L$(STUB_COMMON_PATH) -T$(STUB_LD_SCRIPT) -Wl,--start-group -lgcc -lc -Wl,--end-group

$(STUB_ELF): $(SRCS) $(STUB_COMMON_PATH)/stub_common.ld $(STUB_LD_SCRIPT) $(BUILD_DIR)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -DSTUB_IMAGE=1 -Wl,-Map=$(@:.elf=.map) -o $@ $(LDFLAGS) $(filter %.c, $^)
	$(Q) $(CROSS)size $@

$(STUB_OBJ): $(SRCS) $(STUB_OBJ_DEPS)
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

$(STUB_IMAGE_HDR): $(STUB_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) @printf "#define $(STUB_CHIP)_STUB_BSS_SIZE 0x0" > $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -S $^ | fgrep .bss | awk '{print $$7"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_ENTRY_ADDR 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep stub_main | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "//#define $(STUB_CHIP)_STUB_BUILD_IDF_REV " >> $(STUB_IMAGE_HDR)
	$(Q) cd $(IDF_PATH); git rev-parse --short HEAD >> $(STUB_CHIP_PATH)/$(STUB_IMAGE_HDR)

clean:
	$(Q) rm -rf $(BUILD_DIR) $(STUB_CODE_SECT) $(STUB_DATA_SECT) $(STUB_WRAPPER) $(STUB_IMAGE_HDR)
