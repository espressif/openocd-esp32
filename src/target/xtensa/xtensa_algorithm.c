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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "xtensa.h"
#include "xtensa_algorithm.h"

static int xtensa_algo_init(struct target *target, struct algorithm_run_data *run,
	void *arch_info,
	uint32_t num_args,
	va_list ap);
static int xtensa_algo_cleanup(struct algorithm_run_data *run);
static const uint8_t *xtensa_stub_tramp_get(struct target *target, size_t *size);

const struct algorithm_hw xtensa_algo_hw = {
	.algo_init = xtensa_algo_init,
	.algo_cleanup = xtensa_algo_cleanup,
	.stub_tramp_get = xtensa_stub_tramp_get,
};

static const uint8_t *xtensa_stub_tramp_get(struct target *target, size_t *size)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	static const uint8_t xtensa_stub_tramp_win[] = {
	#include "src/target/xtensa/xtensa_stub_tramp_win.inc"
	};

	if (!xtensa->core_config->windowed) {
		LOG_ERROR(
			"Running stubs is not supported for cores without windowed registers option!");
		return NULL;
	} else {
		*size = sizeof(xtensa_stub_tramp_win);
		return xtensa_stub_tramp_win;
	}
}

static int xtensa_algo_regs_init_start(struct target *target, struct algorithm_run_data *run)
{
	uint32_t stack_addr = run->stub.stack_addr;

	LOG_DEBUG("Check stack addr 0x%x", stack_addr);
	if (stack_addr & 0xFUL) {
		stack_addr &= ~0xFUL;
		LOG_DEBUG("Adjust stack addr to 0x%x", stack_addr);
	}
	stack_addr -= 16;
	init_reg_param(&run->reg_args.params[0], "a0", 32, PARAM_OUT);		/*TODO: move to
										 * tramp */
	init_reg_param(&run->reg_args.params[1], "a1", 32, PARAM_OUT);
	init_reg_param(&run->reg_args.params[2], "a8", 32, PARAM_OUT);
	init_reg_param(&run->reg_args.params[3], "windowbase", 32, PARAM_OUT);	/*TODO: move to
										 * tramp */
	init_reg_param(&run->reg_args.params[4], "windowstart", 32, PARAM_OUT);	/*TODO: move to
										 * tramp */
	init_reg_param(&run->reg_args.params[5], "ps", 32, PARAM_OUT);
	buf_set_u32(run->reg_args.params[0].value, 0, 32, 0);	/* a0 TODO: move to tramp */
	buf_set_u32(run->reg_args.params[1].value, 0, 32, stack_addr);	/* a1 */
	buf_set_u32(run->reg_args.params[2].value, 0, 32, run->stub.entry);	/* a8 */
	buf_set_u32(run->reg_args.params[3].value, 0, 32, 0x0);	/* initial window base TODO: move to
								 * tramp */
	buf_set_u32(run->reg_args.params[4].value, 0, 32, 0x1);	/* initial window start TODO: move
								 * to tramp */
	buf_set_u32(run->reg_args.params[5].value, 0, 32, 0x60025);	/* enable WOE, UM and debug
									* interrupts level (6) */
	return ERROR_OK;
}

static int xtensa_algo_init(struct target *target, struct algorithm_run_data *run,
	void *arch_info,
	uint32_t num_args,
	va_list ap)
{
	enum xtensa_mode core_mode = XT_MODE_ANY;
	char *arg_regs[] = { "a2", "a3", "a4", "a5", "a6" };

	if (num_args > ARRAY_SIZE(arg_regs)) {
		LOG_ERROR("Too many algo user args %u! Max %zu args are supported.",
			num_args, ARRAY_SIZE(arg_regs));
		return ERROR_FAIL;
	}

	if (arch_info) {
		struct xtensa_algorithm *xtensa_algo = arch_info;
		core_mode = xtensa_algo->core_mode;
	}

	run->reg_args.first_user_param = XTENSA_STUB_ARGS_FUNC_START;
	run->reg_args.count = run->reg_args.first_user_param + num_args;
	if (num_args == 0)
		run->reg_args.count++;	/*a2 reg is used as the 1st arg and return code*/
	LOG_DEBUG("reg params count %d (%d/%d).",
		run->reg_args.count,
		run->reg_args.first_user_param,
		num_args);
	run->reg_args.params = calloc(run->reg_args.count, sizeof(struct reg_param));
	assert(run->reg_args.params);

	xtensa_algo_regs_init_start(target, run);

	init_reg_param(&run->reg_args.params[run->reg_args.first_user_param + 0],
		"a2",
		32,
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
			arg_regs[i], 32, PARAM_OUT);
		algorithm_user_arg_set_uint(run, i, arg);
		LOG_DEBUG("Set arg[%d] = %d (%s)", i, arg,
			run->reg_args.params[run->reg_args.first_user_param + i].reg_name);
	}
	struct xtensa_algorithm *ainfo = calloc(1, sizeof(struct xtensa_algorithm));
	assert(ainfo);
	ainfo->core_mode = core_mode;
	run->stub.ainfo = ainfo;
	return ERROR_OK;
}

static int xtensa_algo_cleanup(struct algorithm_run_data *run)
{
	free(run->stub.ainfo);
	for (uint32_t i = 0; i < run->reg_args.count; i++)
		destroy_reg_param(&run->reg_args.params[i]);
	free(run->reg_args.params);
	return ERROR_OK;
}
