/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#ifndef INCLUDED_RTOS_FREERTOS_STACKINGS_H_
#define INCLUDED_RTOS_FREERTOS_STACKINGS_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"

struct freertos_tls_info {
	uint32_t tls_reg;			/* thread local storage register index */
	uint32_t tls_align;			/* thread local storage align */
};

const struct rtos_register_stacking *rtos_freertos_esp32_pick_stacking_info(struct rtos *rtos, int64_t thread_id, int64_t stack_addr);
const struct rtos_register_stacking *rtos_freertos_esp32_s2_pick_stacking_info(struct rtos *rtos, int64_t thread_id, int64_t stack_addr);
const struct rtos_register_stacking *rtos_freertos_esp32_s3_pick_stacking_info(struct rtos *rtos, int64_t thread_id, int64_t stack_addr);

const struct rtos_register_stacking *rtos_freertos_riscv_pick_stacking_info(struct rtos *rtos, int64_t thread_id, int64_t stack_addr);

const struct freertos_tls_info *rtos_freertos_get_tls_info(struct target *target);

#endif	/* ifndef INCLUDED_RTOS_STANDARD_STACKINGS_H_ */
