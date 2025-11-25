// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/espressif/esp_riscv.h>

#define ESP_TARGET_ESP32P4_HW_REV3
#include "esp_stub_config.h"
#undef ESP_TARGET_ESP32P4_HW_REV3

const struct command_map s_cmd_map_hw_rev3[ESP_STUB_CMD_FLASH_MAX_ID + 1] = {
	MAKE_CMD_MAP_ENTRIES
};
