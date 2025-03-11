# SPDX-License-Identifier: GPL-2.0-or-later

# Common Makefile rules to compile the flasher stub program

# Pass V=1 to see the commands being executed by make
ifneq ("$(V)","1")
Q = @
endif

CC = $(CROSS)gcc
SIZE = $(CROSS)size
OBJCOPY = $(CROSS)objcopy
READELF = $(CROSS)readelf
BIN2C = ../../../../../src/helper/bin2char.sh

BUILD_DIR = build
INC_DIR = inc
STUB = stub
SRCS += $(STUB_COMMON_PATH)/stub_flasher.c \
        $(STUB_CHIP_PATH)/stub_flasher_chip.c \
        $(STUB_COMMON_PATH)/stub_logger.c \
        $(STUB_COMMON_PATH)/stub_sha.c \
        $(STUB_COMMON_PATH)/$(STUB_ARCH)/stub_$(STUB_ARCH)_common.c \
        $(IDF_PATH)/components/app_trace/app_trace.c \
        $(IDF_PATH)/components/app_trace/app_trace_util.c \
        $(IDF_PATH)/components/app_trace/app_trace_membufs_proto.c \
        $(IDF_PATH)/components/esp_hw_support/regi2c_ctrl.c

STUB_IMAGE_HDR = $(STUB)_flasher_image.h
STUB_IDF_IMAGE_HDR = $(STUB)_flash_idf_image.h

# Command list
COMMANDS = flash_read flash_write flash_erase flash_erase_check flash_map_get flash_bp_set flash_bp_clear \
	flash_test flash_write_deflated flash_calc_hash flash_clock_configure flash_multi_command flash_idf_binary \
	flash_with_log

ELF_OUTPUTS = $(foreach cmd, $(COMMANDS), $(BUILD_DIR)/$(STUB)_$(cmd).elf)
CODE_SECT_OUTPUTS = $(foreach cmd, $(COMMANDS), $(INC_DIR)/$(STUB)_$(cmd)_code.inc)
DATA_SECT_OUTPUTS = $(foreach cmd, $(COMMANDS), $(INC_DIR)/$(STUB)_$(cmd)_data.inc)
ALL_COMMAND_DEFS = $(foreach cmd, $(COMMANDS), -DCMD_$(shell echo $(cmd) | tr a-z A-Z))

# Loop over each command and generate the variable definitions for each command
$(foreach cmd, $(COMMANDS), \
    $(eval STUB_$(shell echo $(cmd) | tr a-z A-Z)_ELF = $(BUILD_DIR)/$(STUB)_$(cmd).elf) \
    $(eval STUB_$(shell echo $(cmd) | tr a-z A-Z)_OBJ = $(BUILD_DIR)/$(STUB)_$(cmd).o) \
    $(eval STUB_$(shell echo $(cmd) | tr a-z A-Z)_CODE_SECT = $(INC_DIR)/$(STUB)_$(cmd)_code.inc) \
    $(eval STUB_$(shell echo $(cmd) | tr a-z A-Z)_DATA_SECT = $(INC_DIR)/$(STUB)_$(cmd)_data.inc) \
)

CFLAGS += -Wall -Werror -Os \
          -nostdlib -fno-builtin -flto \
          -Wl,-static -g -ffunction-sections -Wl,--gc-sections

INCLUDES += -I. -I$(STUB_COMMON_PATH) -I$(STUB_CHIP_PATH) -I$(STUB_CHIP_ARCH_PATH) \
          -I$(IDF_PATH)/components/$(STUB_ARCH)/include \
          -I$(IDF_PATH)/components/freertos/port/$(STUB_ARCH)/include \
          -I$(IDF_PATH)/components/freertos/esp_additions/arch/$(STUB_ARCH)/include \
          -I$(IDF_PATH)/components/soc/include \
          -I$(IDF_PATH)/components/soc/$(STUB_CHIP)/register \
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
          -I$(IDF_PATH)/components/app_trace/port/include \
          -I$(IDF_PATH)/components/freertos/config/include \
          -I$(IDF_PATH)/components/freertos/config/include/freertos \
          -I$(IDF_PATH)/components/freertos/config/$(STUB_ARCH)/include \
          -I$(IDF_PATH)/components/freertos/FreeRTOS-Kernel/portable/$(STUB_ARCH)/include/freertos

DEFINES += -Dasm=__asm__

CFLAGS += $(INCLUDES) $(DEFINES)

LDFLAGS += -Wl,--start-group -lgcc -lc -Wl,--end-group -Wl,--undefined=s_esp_stub_desc

.PHONY: all clean

all: $(BUILD_DIR) $(INC_DIR) $(ELF_OUTPUTS) $(CODE_SECT_OUTPUTS) $(DATA_SECT_OUTPUTS) $(STUB_IMAGE_HDR) \
	$(STUB_IDF_IMAGE_HDR)

$(BUILD_DIR):
	$(Q) mkdir -p $@

$(INC_DIR):
	$(Q) mkdir -p $@

define BUILD_ELF
$(1): $(SRCS) $(STUB_COMMON_PATH)/stub_common.ld $(BUILD_DIR)
	@echo "  CC   $(SRCS) -> $(1)"
	$(Q) $(CC) $(CFLAGS) $(2) -Wl,-Map=$(BUILD_DIR)/$(notdir $(1:.elf=.map)) -o $(1) $(filter %.c, $(SRCS)) $(LDFLAGS) -L$(STUB_COMMON_PATH) -T$(3)
	$(Q) $(SIZE) $(1)
endef

define BUILD_OBJ
$(1): $(SRCS) $(STUB_OBJ_DEPS)
	@echo "  CC   $(SRCS) -> $(1)"
	$(Q) $(CC) $(CFLAGS) $(2) -c $(filter %.c, $(SRCS)) -o $(1)
endef

define BUILD_SECTION
$(1): $(2)
	@echo "  OBJCOPY $(2) -> $(1)"
	$(Q) $(OBJCOPY) -O binary $(3) $(2) $(BUILD_DIR)/$(4).bin
	$(Q) $(BIN2C) < $(BUILD_DIR)/$(4).bin > $(1)
endef

# Call the BUILD_ELF macro for each command
$(foreach cmd, $(COMMANDS), \
	$(if $(filter $(cmd),flash_with_log), \
		$(eval $(call BUILD_ELF,$(BUILD_DIR)/$(STUB)_$(cmd).elf,-DSTUB_LOG_ENABLE=1 $(ALL_COMMAND_DEFS),$(STUB_LD_SCRIPT))), \
		$(if $(or $(filter $(cmd),flash_map_get flash_bp_set flash_bp_clear flash_multi_command flash_idf_binary)), \
			$(eval $(call BUILD_ELF,$(BUILD_DIR)/$(STUB)_$(cmd).elf,-DCMD_$(shell echo $(cmd) | tr a-z A-Z),$(STUB_IDF_BIN_LD_SCRIPT))), \
			$(eval $(call BUILD_ELF,$(BUILD_DIR)/$(STUB)_$(cmd).elf,-DCMD_$(shell echo $(cmd) | tr a-z A-Z),$(STUB_LD_SCRIPT))) \
		) \
	) \
)

# Call the BUILD_OBJ macro for each command
$(foreach cmd, $(COMMANDS), \
	$(if $(filter $(cmd),flash_with_log), \
		$(eval $(call BUILD_OBJ,$(BUILD_DIR)/$(STUB)_$(cmd).o,-DSTUB_LOG_ENABLE=1 $(ALL_COMMAND_DEFS))), \
		$(eval $(call BUILD_OBJ,$(BUILD_DIR)/$(STUB)_$(cmd).o,-DCMD_$(shell echo $(cmd) | tr a-z A-Z))) \
	) \
)

# Call the BUILD_SECTION macro for each command for code and data sections
$(foreach cmd, $(COMMANDS), \
    $(eval $(call BUILD_SECTION,$(INC_DIR)/$(STUB)_$(cmd)_code.inc,$(BUILD_DIR)/$(STUB)_$(cmd).elf,-j.text,$(STUB)_$(cmd)_code)) \
    $(eval $(call BUILD_SECTION,$(INC_DIR)/$(STUB)_$(cmd)_data.inc,$(BUILD_DIR)/$(STUB)_$(cmd).elf,-j.data,$(STUB)_$(cmd)_data)) \
)

$(STUB_IMAGE_HDR): $(ELF_OUTPUTS)
	@echo "  GENERATE_HEADER  $@"
	$(Q) printf "/* SPDX-License-Identifier: GPL-2.0-or-later */\n\n" > $(STUB_IMAGE_HDR)

	$(Q) $(foreach cmd, $(COMMANDS), \
		$(eval CMD_NAME = $(shell echo $(cmd) | tr a-z A-Z)) \
		$(eval ELF_FILE = $(STUB_$(CMD_NAME)_ELF)) \
		printf "#define ESP_STUB_$(CMD_NAME)_IRAM_ORG 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep .iram_org | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "#define ESP_STUB_$(CMD_NAME)_IRAM_LEN 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep .iram_len | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "#define ESP_STUB_$(CMD_NAME)_DRAM_ORG 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep .dram_org | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "#define ESP_STUB_$(CMD_NAME)_DRAM_LEN 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep .dram_len | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "\n" >> $(STUB_IMAGE_HDR) ; \
	)

	$(Q) $(foreach cmd, $(COMMANDS), \
		$(eval CMD_NAME = $(shell echo $(cmd) | tr a-z A-Z)) \
		$(eval ELF_FILE = $(STUB_$(CMD_NAME)_ELF)) \
		printf "#define ESP_STUB_$(CMD_NAME)_BSS_SIZE 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -S $(ELF_FILE) | fgrep .bss | awk 'NR==1 {print ($$7 "UL"); exit} END {if (NR==0) print "0UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "#define ESP_STUB_$(CMD_NAME)_ENTRY_ADDR 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep stub_main | awk '{print $$2"UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "#define ESP_STUB_$(CMD_NAME)_APPTRACE_CTRL_ADDR 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep s_tracing_ctrl | awk 'NR==1 {print ($$2 "UL"); exit} END {if (NR==0) print "0UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "#define ESP_STUB_$(CMD_NAME)_LOG_ADDR 0x0" >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep s_stub_log_buff | awk 'NR==1 {print ($$2 "UL"); exit} END {if (NR==0) print "0UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "#define ESP_STUB_$(CMD_NAME)_LOG_SIZE " >> $(STUB_IMAGE_HDR) ; \
		$(READELF) -s $(ELF_FILE) | fgrep s_stub_log_buff | awk 'NR==1 {print ($$3 "UL"); exit} END {if (NR==0) print "0UL"}' >> $(STUB_IMAGE_HDR) ; \
		printf "\n" >> $(STUB_IMAGE_HDR) ; \
	)

	$(Q) printf "#define ESP_STUB_STACK_SIZE $(STUB_STACK_SIZE)\n\n" >> $(STUB_IMAGE_HDR)

	$(Q) printf "#include <stdint.h>\n\n" >> $(STUB_IMAGE_HDR)
	$(Q) $(foreach cmd, $(COMMANDS), \
		printf "static const uint8_t s_esp_flasher_stub_$(cmd)_code[] = {\n" >> $(STUB_IMAGE_HDR) ; \
		printf "#include \"contrib/loaders/flash/espressif/$(STUB_CHIP)/$(INC_DIR)/stub_$(cmd)_code.inc\"\n" >> $(STUB_IMAGE_HDR) ; \
		printf "};\n" >> $(STUB_IMAGE_HDR) ; \
		printf "static const uint8_t s_esp_flasher_stub_$(cmd)_data[] = {\n" >> $(STUB_IMAGE_HDR) ; \
		printf "#include \"contrib/loaders/flash/espressif/$(STUB_CHIP)/$(INC_DIR)/stub_$(cmd)_data.inc\"\n" >> $(STUB_IMAGE_HDR) ; \
		printf "};\n" >> $(STUB_IMAGE_HDR) ; \
	)

	$(Q) @printf "\n/*\n#define $(STUB_CHIP)_STUB_BUILD_IDF_REV " >> $(STUB_IMAGE_HDR)
	$(Q) cd $(IDF_PATH); git rev-parse --short HEAD >> $(STUB_CHIP_PATH)/$(STUB_IMAGE_HDR)
	$(Q) @printf "*/\n" >> $(STUB_IMAGE_HDR)

$(STUB_IDF_IMAGE_HDR): $(ELF_OUTPUTS)
	@echo "  GENERATE_IDF_HEADER  $@"
	$(Q) printf "/* SPDX-License-Identifier: GPL-2.0-or-later */\n\n" > $(STUB_IDF_IMAGE_HDR)

	$(eval CMD_NAME = FLASH_IDF_BINARY)
	$(eval ELF_FILE = $(STUB_$(CMD_NAME)_ELF))
	$(Q) printf "#define OPENOCD_STUB_BSS_SIZE 0x0" >> $(STUB_IDF_IMAGE_HDR)
	$(Q) $(READELF) -S $(ELF_FILE) | fgrep .bss | awk 'NR==1 {print ($$7 "UL"); exit} END {if (NR==0) print "0UL"}' >> $(STUB_IDF_IMAGE_HDR)
	$(Q) printf "#define OPENOCD_STUB_STACK_SIZE $(STUB_STACK_SIZE)\n" >> $(STUB_IDF_IMAGE_HDR)
	$(Q) printf "#define OPENOCD_STUB_PARAM_SIZE 512\n" >> $(STUB_IDF_IMAGE_HDR)
	$(Q) printf "#define OPENOCD_STUB_BP_SECTOR_SIZE 4096\n" >> $(STUB_IDF_IMAGE_HDR)

clean:
	@echo "  CLEAN"
	$(Q) rm -rf $(BUILD_DIR) $(STUB_IMAGE_HDR) $(STUB_IDF_IMAGE_HDR) $(INC_DIR)/*.inc
