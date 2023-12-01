# SPDX-License-Identifier: GPL-2.0-or-later

# Common Makefile rules to compile the flasher stub program

# Pass V=1 to see the commands being executed by make
ifneq ("$(V)","1")
Q = @
endif

STUB = stub_flasher
SRCS += $(STUB_COMMON_PATH)/stub_flasher.c \
	$(STUB_CHIP_PATH)/stub_flasher_chip.c \
	$(STUB_COMMON_PATH)/stub_logger.c \
	$(STUB_COMMON_PATH)/stub_sha.c \
	$(STUB_COMMON_PATH)/$(STUB_ARCH)/stub_$(STUB_ARCH)_common.c \
	$(IDF_PATH)/components/app_trace/app_trace.c \
	$(IDF_PATH)/components/app_trace/app_trace_util.c \
	$(IDF_PATH)/components/app_trace/app_trace_membufs_proto.c \
	$(IDF_PATH)/components/esp_hw_support/regi2c_ctrl.c

BUILD_DIR = build
BIN2C = ../../../../../src/helper/bin2char.sh

STUB_ELF = $(BUILD_DIR)/$(STUB).elf
STUB_OBJ = $(BUILD_DIR)/$(STUB).o
STUB_CODE_SECT = $(STUB)_code.inc
STUB_DATA_SECT = $(STUB)_data.inc
STUB_IMAGE_HDR = $(STUB)_image.h

# Log enabled image dependencies
STUB_WLOG_ELF = $(BUILD_DIR)/$(STUB)_wlog.elf
STUB_WLOG_OBJ = $(BUILD_DIR)/$(STUB)_wlog.o
STUB_CODE_WLOG_SECT = $(STUB)_code_wlog.inc
STUB_DATA_WLOG_SECT = $(STUB)_data_wlog.inc
STUB_IMAGE_WLOG_HDR = $(STUB)_image_wlog.h

.PHONY: all clean

all: $(STUB_ELF) $(STUB_IMAGE_HDR) $(STUB_CODE_SECT) $(STUB_DATA_SECT) \
	$(STUB_WLOG_ELF) $(STUB_CODE_WLOG_SECT) $(STUB_DATA_WLOG_SECT) $(STUB_IMAGE_WLOG_HDR)

$(BUILD_DIR):
	$(Q) mkdir $@

CFLAGS += -Wall -Werror -Os \
         -nostdlib -fno-builtin -flto \
         -Wl,-static -g -ffunction-sections -Wl,--gc-sections

INCLUDES += -I. -I$(STUB_COMMON_PATH) -I$(STUB_CHIP_PATH) -I$(STUB_CHIP_ARCH_PATH) \
		-I$(IDF_PATH)/components/$(STUB_ARCH)/include \
		-I$(IDF_PATH)/components/freertos/port/$(STUB_ARCH)/include \
		-I$(IDF_PATH)/components/freertos/esp_additions/arch/$(STUB_ARCH)/include \
		-I$(IDF_PATH)/components/soc/include \
		-I$(IDF_PATH)/components/driver/include \
		-I$(IDF_PATH)/components/log/include \
		-I$(IDF_PATH)/components/heap/include \
		-I$(IDF_PATH)/components/bootloader_support/include \
		-I$(IDF_PATH)/components/efuse/include \
		-I$(IDF_PATH)/components/hal/include \
		-I$(IDF_PATH)/components/hal/platform_port/include \
		-I$(IDF_PATH)/components/spi_flash/include/spi_flash \
		-I$(IDF_PATH)/components/newlib/platform_include \
		-I$(IDF_PATH)/components/esp_timer/include \
		-I$(IDF_PATH)/components/esp_rom/include \
		-I$(IDF_PATH)/components/esp_common/include \
		-I$(IDF_PATH)/components/esp_system/include \
		-I$(IDF_PATH)/components/esp_system/port/public_compat \
		-I$(IDF_PATH)/components/esp_hw_support/include \
		-I$(IDF_PATH)/components/esp_hw_support/include/soc \
		-I$(IDF_PATH)/components/esp_hw_support/include/esp_private \
		-I$(IDF_PATH)/components/esp_hw_support/port/include \
		-I$(IDF_PATH)/components/freertos/esp_additions/include/freertos \
		-I$(IDF_PATH)/components/freertos/FreeRTOS-Kernel/include \
		-I$(IDF_PATH)/components/freertos/FreeRTOS-Kernel/portable/$(STUB_ARCH)/include \
		-I$(IDF_PATH)/components/spi_flash/include \
		-I$(IDF_PATH)/components/spi_flash/private_include \
		-I$(IDF_PATH)/components/esp_system/port/include/private \
		-I$(IDF_PATH)/components/app_trace/include \
		-I$(IDF_PATH)/components/app_trace/private_include \
		-I$(IDF_PATH)/components/app_trace/port/include

DEFINES += -Dasm=__asm__

CFLAGS += $(INCLUDES) $(DEFINES)

LDFLAGS += -L$(STUB_COMMON_PATH) -T$(STUB_LD_SCRIPT) -Wl,--start-group -lgcc -lc -Wl,--end-group

$(STUB_ELF): $(SRCS) $(STUB_COMMON_PATH)/stub_common.ld $(STUB_LD_SCRIPT) $(BUILD_DIR)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -DSTUB_IMAGE=1 -Wl,-Map=$(@:.elf=.map) -o $@ $(filter %.c, $^) $(LDFLAGS)
	$(Q) $(CROSS)size $@

$(STUB_WLOG_ELF): $(SRCS) $(STUB_COMMON_PATH)/stub_common.ld $(STUB_LD_SCRIPT) $(BUILD_DIR)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -DSTUB_IMAGE=1 -DSTUB_LOG_ENABLE=1 -Wl,-Map=$(@:.elf=.map) -o $@ $(filter %.c, $^) $(LDFLAGS)
	$(Q) $(CROSS)size $@

$(STUB_OBJ): $(SRCS) $(STUB_OBJ_DEPS)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -c $(filter %.c, $^) -o $@

$(STUB_WLOG_OBJ): $(SRCS) $(STUB_OBJ_DEPS)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)gcc $(CFLAGS) -DSTUB_LOG_ENABLE=1 -c $(filter %.c, $^) -o $@

$(STUB_CODE_SECT): $(STUB_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)objcopy -O binary -j.text $^ $(BUILD_DIR)/$(STUB)_code.bin
	$(Q) $(BIN2C) < $(BUILD_DIR)/$(STUB)_code.bin > $@

$(STUB_CODE_WLOG_SECT): $(STUB_WLOG_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)objcopy -O binary -j.text $^ $(BUILD_DIR)/$(STUB)_code_wlog.bin
	$(Q) $(BIN2C) < $(BUILD_DIR)/$(STUB)_code_wlog.bin > $@

$(STUB_DATA_SECT): $(STUB_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)objcopy -O binary -j.data $^ $(BUILD_DIR)/$(STUB)_data.bin
	$(Q) $(BIN2C) < $(BUILD_DIR)/$(STUB)_data.bin > $@

$(STUB_DATA_WLOG_SECT): $(STUB_WLOG_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) $(CROSS)objcopy -O binary -j.data $^ $(BUILD_DIR)/$(STUB)_data_wlog.bin
	$(Q) $(BIN2C) < $(BUILD_DIR)/$(STUB)_data_wlog.bin > $@

$(STUB_IMAGE_HDR): $(STUB_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) @printf "/* SPDX-License-Identifier: GPL-2.0-or-later */\n\n" > $(STUB_IMAGE_HDR)
	$(Q) @printf "#define $(STUB_CHIP)_STUB_BSS_SIZE 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -S $^ | fgrep .bss | awk '{print $$7"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_IRAM_ORG 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep .iram_org | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_IRAM_LEN 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep .iram_len | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_DRAM_ORG 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep .dram_org | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_DRAM_LEN 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep .dram_len | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_ENTRY_ADDR 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep stub_main | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_APPTRACE_CTRL_ADDR 0x0" >> $(STUB_IMAGE_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep s_tracing_ctrl | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR)
	$(Q) @printf "\n" >> $(STUB_CHIP_PATH)/$(STUB_IMAGE_HDR)
	$(Q) @printf "/*#define $(STUB_CHIP)_STUB_BUILD_IDF_REV " >> $(STUB_IMAGE_HDR)
	$(Q) cd $(IDF_PATH); git rev-parse --short HEAD >> $(STUB_CHIP_PATH)/$(STUB_IMAGE_HDR)
	$(Q) @printf "*/\n" >> $(STUB_IMAGE_HDR)

$(STUB_IMAGE_WLOG_HDR): $(STUB_WLOG_ELF)
	@echo "  CC   $^ -> $@"
	$(Q) @printf "/* SPDX-License-Identifier: GPL-2.0-or-later */\n\n" > $(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "#define $(STUB_CHIP)_STUB_WLOG_BSS_SIZE 0x0" >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) $(CROSS)readelf -S $^ | fgrep .bss | awk '{print $$7"UL"}' >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_WLOG_LOG_ADDR 0x0" >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep s_stub_log_buff | awk '{print $$2"UL"}' >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_WLOG_LOG_SIZE " >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep s_stub_log_buff | awk '{print $$3"UL"}' >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_WLOG_ENTRY_ADDR 0x0" >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep stub_main | awk '{print $$2"UL"}' >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "\\n#define $(STUB_CHIP)_STUB_WLOG_APPTRACE_CTRL_ADDR 0x0" >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) $(CROSS)readelf -s $^ | fgrep s_tracing_ctrl | awk '{print $$2"UL"}' >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "\n" >> $(STUB_CHIP_PATH)/$(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "/*#define $(STUB_CHIP)_STUB_WLOG_BUILD_IDF_REV " >> $(STUB_IMAGE_WLOG_HDR)
	$(Q) cd $(IDF_PATH); git rev-parse --short HEAD >> $(STUB_CHIP_PATH)/$(STUB_IMAGE_WLOG_HDR)
	$(Q) @printf "*/\n" >> $(STUB_IMAGE_WLOG_HDR)

clean:
	$(Q) rm -rf $(BUILD_DIR) $(STUB_CODE_SECT) $(STUB_DATA_SECT) $(STUB_WRAPPER) $(STUB_IMAGE_HDR) \
		$(STUB_CODE_WLOG_SECT) $(STUB_DATA_WLOG_SECT) $(STUB_IMAGE_WLOG_HDR)
