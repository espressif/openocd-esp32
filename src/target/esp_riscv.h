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
#include "riscv/riscv_algorithm.h"
#include "esp_riscv_apptrace.h"
#include "esp.h"

struct esp_riscv_common {
	/* should be first, will be accessed by riscv generic code */
	riscv_info_t riscv;
	struct esp_riscv_apptrace_info apptrace;
	const struct algorithm_hw *algo_hw;
	struct esp_dbg_stubs dbg_stubs;
	struct esp_semihost_ops *semi_ops;
};

static inline struct esp_riscv_common *target_to_esp_riscv(const struct target *target)
{
	return target->arch_info;
}

static inline int esp_riscv_init_target_info(struct command_context *cmd_ctx, struct target *target,
	struct esp_riscv_common *esp_riscv, int (*on_reset)(struct target *),
	const struct esp_semihost_ops *semi_ops)
{
	esp_riscv->apptrace.hw = &esp_riscv_apptrace_hw;
	esp_riscv->algo_hw = &riscv_algo_hw;
	esp_riscv->semi_ops = (struct esp_semihost_ops *)semi_ops;
	int ret = riscv_init_target_info(cmd_ctx, target, &esp_riscv->riscv);
	esp_riscv->riscv.on_reset = on_reset;
	return ret;
}

#endif	/* _ESP_RISCV_H */
