/***************************************************************************
 *   Module to run arbitrary code on RISCV using OpenOCD                   *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_RISCV_ALGO_H
#define OPENOCD_TARGET_ESP_RISCV_ALGO_H

#include <target/espressif/esp_algorithm.h>
#include <target/riscv/riscv.h>

/** Index of the first user-defined algo arg. @see algorithm_stub */
#define ESP_RISCV_STUB_ARGS_FUNC_START  2

/**
 * RISCV algorithm data.
 */
struct esp_riscv_algorithm {
	/** Registers with numbers below and including this number will be backuped before algo start.
	 * Set to GDB_REGNO_COUNT-1 to save all existing registers. @see enum gdb_regno. */
	size_t max_saved_reg;
	uint64_t saved_registers[RISCV_MAX_REGISTERS];
	bool valid_saved_registers[RISCV_MAX_REGISTERS];
};

extern const struct algorithm_hw riscv_algo_hw;

#endif	/* OPENOCD_TARGET_ESP_RISCV_ALGO_H */
