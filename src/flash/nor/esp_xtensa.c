/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP Xtensa flash driver for OpenOCD                                   *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "esp_xtensa.h"
#include <target/espressif/esp_xtensa_apptrace.h>
#include <target/xtensa/xtensa_algorithm.h>

#define ESP_XTENSA_FLASH_MIN_OFFSET  0x1000	/* protect secure boot digest data */

static const struct esp_flash_apptrace_hw s_esp_xtensa_flash_apptrace_hw = {
	.data_len_read = esp_xtensa_apptrace_data_len_read,
	.data_read = esp_xtensa_apptrace_data_read,
	.ctrl_reg_read = esp_xtensa_apptrace_ctrl_reg_read,
	.ctrl_reg_write = esp_xtensa_apptrace_ctrl_reg_write,
	.usr_block_write = esp_xtensa_apptrace_usr_block_write,
	.usr_block_get = esp_apptrace_usr_block_get,
	.block_max_size_get = esp_xtensa_apptrace_block_max_size_get,
	.usr_block_max_size_get = esp_xtensa_apptrace_usr_block_max_size_get,
};

int esp_xtensa_flash_init(struct esp_xtensa_flash_bank *esp_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct algorithm_run_data *run,
		uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank))
{
	memset(esp_info, 0, sizeof(*esp_info));
	int ret = esp_algo_flash_init(&esp_info->esp, sec_sz, run_func_image, is_irom_address,
		is_drom_address, get_stub, &s_esp_xtensa_flash_apptrace_hw,
		&xtensa_algo_hw);
	esp_info->esp.flash_min_offset = ESP_XTENSA_FLASH_MIN_OFFSET;
	return ret;
}
