/***************************************************************************
 *   Copyright (c) 2020 Espressif Systems (Shanghai) Co. Ltd.                                *
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

#pragma once

#include "target.h"
#include "command.h"
#include "xtensa.h"
#include "xtensa_semihosting.h"
#include "semihosting_common.h"
#include "esp_xtensa.h"
#include "stdbool.h"

#define SEMIHOSTING_SYS_DRV_INFO 0xE0
#define XT_FILE_FLAGS_MASK 0xFFF00000

int esp_xtensa_semihosting_init(struct target *target);
int esp_xtensa_semihosting(struct target *target);
char *esp_xtensa_semihosting_get_file_name(struct target *target, uint64_t addr_fn, size_t len, uint32_t * mode);
