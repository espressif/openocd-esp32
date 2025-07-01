/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Xtensa chips common definitions for flasher stub                      *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_XTENSA_COMMON_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_XTENSA_COMMON_H

uint8_t stub_get_insn_size(uint8_t *insn);
uint8_t stub_get_max_insn_size(void);
uint32_t stub_get_break_insn(uint8_t insn_sz);
uint32_t stub_get_coreid(void);
uint64_t stub_get_time(void);
uint32_t stub_flash_get_id(void);

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_XTENSA_COMMON_H */
