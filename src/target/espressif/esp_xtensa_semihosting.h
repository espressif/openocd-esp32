/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (c) 2020 Espressif Systems (Shanghai) Co. Ltd.              *
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
