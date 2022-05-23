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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/target.h>
#include <target/riscv/riscv.h>
#include "esp_riscv_algorithm.h"

static int esp_riscv_algo_init(struct target *target, struct algorithm_run_data *run,
	void *arch_info,
	uint32_t num_args,
	va_list ap);
static int esp_riscv_algo_cleanup(struct algorithm_run_data *run);
static const uint8_t *esp_riscv_stub_tramp_get(struct target *target, size_t *size);

const struct algorithm_hw riscv_algo_hw = {
	.algo_init = esp_riscv_algo_init,
	.algo_cleanup = esp_riscv_algo_cleanup,
	.stub_tramp_get = esp_riscv_stub_tramp_get,
};

static const uint8_t *esp_riscv_stub_tramp_get(struct target *target, size_t *size)
{
	static const uint8_t s_riscv_stub_tramp[] = {
	#include "src/target/espressif/esp_riscv_stub_tramp.inc"
	};

	*size = sizeof(s_riscv_stub_tramp);
	return s_riscv_stub_tramp;
}

static int esp_riscv_algo_regs_init_start(struct target *target, struct algorithm_run_data *run)
{
	uint32_t stack_addr = run->stub.stack_addr;
	int xlen = riscv_xlen(target);

	LOG_DEBUG("Check stack addr 0x%x", stack_addr);
	if (stack_addr & 0xFUL) {
		stack_addr &= ~0xFUL;
		LOG_DEBUG("Adjust stack addr to 0x%x", stack_addr);
	}
	stack_addr -= 16;
	init_reg_param(&run->reg_args.params[0], "sp", xlen, PARAM_OUT);
	init_reg_param(&run->reg_args.params[1], "a7", xlen, PARAM_OUT);
	buf_set_u64(run->reg_args.params[0].value, 0, xlen, stack_addr);
	buf_set_u64(run->reg_args.params[1].value, 0, xlen, run->stub.entry);
	return ERROR_OK;
}

static int esp_riscv_algo_init(struct target *target, struct algorithm_run_data *run,
	void *arch_info,
	uint32_t num_args,
	va_list ap)
{
	int xlen = riscv_xlen(target);
	char *arg_regs[] = { "a0", "a1", "a2", "a3", "a4", "a5", "a6" };

	if (num_args > ARRAY_SIZE(arg_regs)) {
		LOG_ERROR("Too many algo user args %u! Max %zu args are supported.",
			num_args, ARRAY_SIZE(arg_regs));
		return ERROR_FAIL;
	}

	run->reg_args.first_user_param = ESP_RISCV_STUB_ARGS_FUNC_START;
	run->reg_args.count = run->reg_args.first_user_param + num_args;
	run->reg_args.params = calloc(run->reg_args.count, sizeof(struct reg_param));
	assert(run->reg_args.params);

	esp_riscv_algo_regs_init_start(target, run);

	init_reg_param(&run->reg_args.params[run->reg_args.first_user_param + 0],
		"a0",
		xlen,
		PARAM_IN_OUT);

	if (num_args > 0) {
		uint32_t arg = va_arg(ap, uint32_t);
		algorithm_user_arg_set_uint(run, 0, arg);
		LOG_DEBUG("Set arg[0] = %d (%s)", arg,
			run->reg_args.params[run->reg_args.first_user_param + 0].reg_name);
	} else {
		algorithm_user_arg_set_uint(run, 0, 0);
	}

	for (uint32_t i = 1; i < num_args; i++) {
		uint32_t arg = va_arg(ap, uint32_t);
		init_reg_param(&run->reg_args.params[run->reg_args.first_user_param + i],
			arg_regs[i], xlen, PARAM_OUT);
		algorithm_user_arg_set_uint(run, i, arg);
		LOG_DEBUG("Set arg[%d] = %d (%s)", i, arg,
			run->reg_args.params[run->reg_args.first_user_param + i].reg_name);
	}
	struct esp_riscv_algorithm *ainfo = calloc(1, sizeof(struct esp_riscv_algorithm));
	assert(ainfo);
	/* backup all regs */
	ainfo->max_saved_reg = GDB_REGNO_COUNT - 1;
	run->stub.ainfo = ainfo;
	return ERROR_OK;
}

static int esp_riscv_algo_cleanup(struct algorithm_run_data *run)
{
	free(run->stub.ainfo);
	for (uint32_t i = 0; i < run->reg_args.count; i++)
		destroy_reg_param(&run->reg_args.params[i]);
	free(run->reg_args.params);
	return ERROR_OK;
}
