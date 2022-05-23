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

#ifndef OPENOCD_TARGET_ESP_XTENSA_SEMIHOSTING_H
#define OPENOCD_TARGET_ESP_XTENSA_SEMIHOSTING_H

#include <unistd.h>
#include <target/target.h>
#include <helper/command.h>
#include <target/xtensa/xtensa.h>
#include <target/semihosting_common.h>
#include "esp_xtensa.h"
#include "esp_semihosting.h"
#include "stdbool.h"

int esp_xtensa_semihosting_init(struct target *target);
int esp_xtensa_semihosting(struct target *target, int *retval);

#endif	/* OPENOCD_TARGET_ESP_XTENSA_SEMIHOSTING_H */
