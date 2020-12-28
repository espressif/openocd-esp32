/***************************************************************************
 *   ESP RISCV common definitions for OpenOCD                              *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef _ESP_RISCV_H
#define _ESP_RISCV_H

#include "target.h"
#include "riscv/riscv.h"
#include "esp_riscv_apptrace.h"
#include "esp.h"

struct esp_riscv_common {
	/* should be first, will be accessed by riscv generic code */
	riscv_info_t riscv;
	struct esp_riscv_apptrace_info apptrace;
	struct esp_dbg_stubs dbg_stubs;
};

static inline struct esp_riscv_common *target_to_esp_riscv(const struct target *target)
{
	return target->arch_info;
}

static inline int esp_riscv_init_target_info(struct command_context *cmd_ctx,
	struct target *target,
	struct esp_riscv_common *esp_riscv)
{
	esp_riscv->apptrace.hw = &esp_riscv_apptrace_hw;
	return riscv_init_target_info(cmd_ctx, target, &esp_riscv->riscv);
}

#endif	/* _ESP_RISCV_H */
