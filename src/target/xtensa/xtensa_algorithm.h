/***************************************************************************
 *   Module to run arbitrary code on Xtensa using OpenOCD                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
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

#ifndef OPENOCD_TARGET_XTENSA_ALGO_H
#define OPENOCD_TARGET_XTENSA_ALGO_H

#include <target/espressif/esp_algorithm.h>
#include "xtensa.h"

/** Index of the first user-defined algo arg. @see algorithm_stub */
#define XTENSA_STUB_ARGS_FUNC_START             6

/**
 * Xtensa algorithm data.
 */
struct xtensa_algorithm {
	/** User can set this to specify which core mode algorithm should be run in. */
	enum xtensa_mode core_mode;
	/** Used internally to backup and restore debug_reason. */
	enum target_debug_reason ctx_debug_reason;
};

extern const struct algorithm_hw xtensa_algo_hw;

#endif	/* OPENOCD_TARGET_XTENSA_ALGO_H */
