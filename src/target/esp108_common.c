/***************************************************************************
 *   ESP108 and ESP32 targets for OpenOCD                                  *
 *   Copyright (C) 2016-2017 Espressif Systems Ltd.                        *
 *                                                                         *
 *   Derived from original ESP8266 target.                                 *
 *   Copyright (C) 2015 by Angus Gratton                                   *
 *   gus@projectgus.com                                                    *
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

#include "esp108_common.h"
#include "esp108_dbg_regs.h"

//Convert a register index that's indexed relative to windowbase, to the real address.
enum xtensa_reg_idx windowbase_offset_to_canonical(const enum xtensa_reg_idx reg, const int windowbase)
{
	int idx;
	if (reg >= XT_REG_IDX_AR0 && reg <= XT_REG_IDX_AR63) {
		idx = reg - XT_REG_IDX_AR0;
	}
	else if (reg >= XT_REG_IDX_A0 && reg <= XT_REG_IDX_A15) {
		idx = reg - XT_REG_IDX_A0;
	}
	else {
		LOG_ERROR("Error: can't convert register %d to non-windowbased register!\n", reg);
		return -1;
	}
	return ((idx + (windowbase * 4)) & 63) + XT_REG_IDX_AR0;
}

enum xtensa_reg_idx canonical_to_windowbase_offset(const enum xtensa_reg_idx reg, const int windowbase)
{
	return windowbase_offset_to_canonical(reg, -windowbase);
}

int regReadable(int flags, int cpenable)
{
	if (flags&XT_REGF_NOREAD) return 0;
	if ((flags&XT_REGF_COPROC0) && (cpenable&(1 << 0)) == 0) return 0;
	return 1;
}


/*
The TDI pin is also used as a flash Vcc bootstrap pin. If we reset the CPU externally, the last state of the TDI pin can
allow the power to an 1.8V flash chip to be raised to 3.3V, or the other way around. Users can use the
esp108 flashbootstrap command to set a level, and this routine will make sure the tdi line will return to
that when the jtag port is idle.
*/
void esp108_queue_tdi_idle(struct target *target) {
	struct esp108_common *esp108 = (struct esp108_common*)target->arch_info;
	static uint8_t value;
	uint8_t t[4] = { 0, 0, 0, 0 };

	if (esp108->flashBootstrap == FBS_TMSLOW) {
		//Make sure tdi is 0 at the exit of queue execution
		value = 0;
	}
	else if (esp108->flashBootstrap == FBS_TMSHIGH) {
		//Make sure tdi is 1 at the exit of queue execution
		value = 1;
	}
	else {
		return;
	}

	//Scan out 1 bit, do not move from IRPAUSE after we're done.
	buf_set_u32(t, 0, 1, value);
	jtag_add_plain_ir_scan(1, t, NULL, TAP_IRPAUSE);
}

void esp108_add_set_ir(struct target *target, uint8_t value)
{
	uint8_t t[4];
	struct scan_field field;
	memset(&field, 0, sizeof field);
	field.num_bits = target->tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, value);
	jtag_add_ir_scan(target->tap, &field, TAP_IDLE);
}

void esp108_add_dr_scan(struct target *target, int len, const uint8_t *src, uint8_t *dest, tap_state_t endstate)
{
	struct scan_field field;
	memset(&field, 0, sizeof field);
	field.num_bits = len;
	field.out_value = src;
	field.in_value = dest;
	jtag_add_dr_scan(target->tap, 1, &field, endstate);
}

void esp108_queue_nexus_reg_write(struct target *target, const uint8_t reg, const uint32_t value)
{
	uint8_t regdata = (reg << 1) | 1;
	uint8_t valdata[] = { value, value >> 8, value >> 16, value >> 24 };
	esp108_add_set_ir(target, TAPINS_NARSEL);
	esp108_add_dr_scan(target, TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	esp108_add_dr_scan(target, TAPINS_NARSEL_DATALEN, valdata, NULL, TAP_IDLE);
}

void esp108_queue_nexus_reg_read(struct target *target, const uint8_t reg, uint8_t *value)
{
	uint8_t regdata = (reg << 1) | 0;
	uint8_t dummy[4] = { 0, 0, 0, 0 };
	esp108_add_set_ir(target, TAPINS_NARSEL);
	esp108_add_dr_scan(target, TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	esp108_add_dr_scan(target, TAPINS_NARSEL_DATALEN, dummy, value, TAP_IDLE);
}

//Set the PWRCTL TAP register to a value
void esp108_queue_pwrctl_set(struct target *target, uint8_t value)
{
	esp108_add_set_ir(target, TAPINS_PWRCTL);
	esp108_add_dr_scan(target, TAPINS_PWRCTL_LEN, &value, NULL, TAP_IDLE);
}

//Read the PWRSTAT TAP register and clear the XWASRESET bits.
void esp108_queue_pwrstat_readclear(struct target *target, uint8_t *value)
{
	const uint8_t pwrstatClr = PWRSTAT_DEBUGWASRESET | PWRSTAT_COREWASRESET;
	esp108_add_set_ir(target, TAPINS_PWRSTAT);
	esp108_add_dr_scan(target, TAPINS_PWRCTL_LEN, &pwrstatClr, value, TAP_IDLE);
}

void esp108_queue_exec_ins(struct target *target, int32_t ins)
{
	esp108_queue_nexus_reg_write(target, NARADR_DIR0EXEC, ins);
}


uint32_t esp108_reg_get(struct reg *reg)
{
	return *((uint32_t*)reg->value);
}

void esp108_reg_set(struct reg *reg, uint32_t value)
{
	uint32_t oldval;
	oldval = *((uint32_t*)reg->value);
	if (oldval == value) return;
	*((uint32_t*)reg->value) = value;
	reg->dirty = 1;
}

int esp108_do_checkdsr(struct target *target, const char *function, const int line)
{
	uint8_t dsr[4];
	int res;
	int needclear = 0;
	esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
	esp108_queue_tdi_idle(target);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("%s: %s (line %d): reading DSR failed!", target->cmd_name, function, line);
		return ERROR_FAIL;
	}
	if (intfromchars(dsr)&OCDDSR_EXECBUSY) {
		LOG_ERROR("%s: %s (line %d): DSR (%08X) indicates target still busy!", target->cmd_name, function, line, intfromchars(dsr));
		needclear = 1;
	}
	if (intfromchars(dsr)&OCDDSR_EXECEXCEPTION) {
		LOG_ERROR("%s: %s (line %d): DSR (%08X) indicates DIR instruction generated an exception!", target->cmd_name, function, line, intfromchars(dsr));
		needclear = 1;
	}
	if (intfromchars(dsr)&OCDDSR_EXECOVERRUN) {
		LOG_ERROR("%s: %s (line %d): DSR (%08X) indicates DIR instruction generated an overrun!", target->cmd_name, function, line, intfromchars(dsr));
		needclear = 1;
	}
	if (needclear) {
		esp108_queue_nexus_reg_write(target, NARADR_DSR, OCDDSR_EXECEXCEPTION | OCDDSR_EXECOVERRUN);
		res = jtag_execute_queue();
		if (res != ERROR_OK) {
			LOG_ERROR("%s: %s (line %d): clearing DSR failed!", target->cmd_name, function, line);
		}
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

unsigned int xtensa_read_dsr(struct target *target)
{
	uint8_t dsr[4];
	int res;

	esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_ENABLEOCD);
	esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
	esp108_queue_tdi_idle(target);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
	{
		LOG_ERROR("%s: Failed to read NARADR_DSR. Can't halt.", target->cmd_name);
		return res;
	}
	uint32_t regval = intfromchars(dsr);
	esp108_queue_nexus_reg_write(target, NARADR_DSR, regval);
	esp108_queue_tdi_idle(target);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
	{
		LOG_ERROR("%s: Failed to write NARADR_DSR. Can't halt.", target->cmd_name);
		return res;
	}

	return intfromchars(dsr);
}

int xtensa_get_core_reg(struct reg *reg)
{
	//We don't need this because we read all registers on halt anyway.
	struct esp108_common *esp108 = (struct esp108_common*)reg->arch_info;
	struct target *target = esp108->target;
	if (target->state != TARGET_HALTED) return ERROR_TARGET_NOT_HALTED;
	return ERROR_OK;
}

int xtensa_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct esp108_common *esp108 = (struct esp108_common*)reg->arch_info;
	struct target *target = esp108->target;

	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED) return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = 1;
	reg->valid = 1;
	return ERROR_OK;
}


//Stub
int xtensa_examine(struct target *target)
{
	target_set_examined(target);
	return ERROR_OK;
}

uint32_t xtensa_read_reg_direct(struct target *target, uint8_t reg)
{
	uint32_t result = 0xdeadface;
	uint8_t dsr[4];
	//XT_REG_IDX_DEBUGCAUSE

	esp108_queue_nexus_reg_read(target, reg, dsr);
	esp108_queue_tdi_idle(target);
	int res = jtag_execute_queue();
	if (res != ERROR_OK) return result;
	result = intfromchars(dsr);
	return result;
}

int read_reg_direct(struct target *target, uint8_t addr)
{
	uint8_t dsr[4];
	esp108_queue_nexus_reg_read(target, addr, dsr);
	esp108_queue_tdi_idle(target);
	jtag_execute_queue();
	return intfromchars(dsr);
}


int xtensa_write_uint32(struct target *target, uint32_t addr, uint32_t val)
{

	return target->type->write_memory(target, addr, 4, 1, (uint8_t*)&val);
}

int xtensa_write_uint32_list(struct target *target, const uint32_t* addr_value_pairs_list, size_t count)
{
	int res;
	for (size_t i = 0; i < count; ++i)
	{
		res = xtensa_write_uint32(target, addr_value_pairs_list[2 * i], addr_value_pairs_list[2 * i + 1]);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: error writing to %08x", __func__, addr_value_pairs_list[2 * i]);
			return res;
		}
	}
	return ERROR_OK;
}

int xtensa_start_algorithm_generic(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	void *arch_info, struct reg_cache *core_cache)
{
	struct xtensa_algorithm *algorithm_info = arch_info;
	enum xtensa_mode core_mode;;
	int retval = ERROR_OK;
	int usr_ps = 0;

	/* NOTE: xtensa_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Target not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache */
	for (unsigned i = 0; i < core_cache->num_regs; i++) {
		algorithm_info->context[i] = esp108_reg_get(&core_cache->reg_list[i]);
	}
	/* write mem params */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_IN) {
			retval = target_write_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	/* write reg params */
	for (int i = 0; i < num_reg_params; i++) {
		struct reg *reg = register_get_by_name(core_cache, reg_params[i].reg_name, 0);
		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
				reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (memcmp(reg_params[i].reg_name, "ps", 3)) {
			usr_ps = 1;
		}
		esp108_reg_set(reg, buf_get_u32(reg_params[i].value, 0, 32));
		reg->valid = 1;
	}
	uint32_t ps = esp108_reg_get(&core_cache->reg_list[XT_REG_IDX_PS]);
	uint32_t new_ps = ps;
	// ignore custom core mode if custom PS value is specified
	if (!usr_ps) {
		core_mode = XT_PS_RING_GET(ps);
		if (algorithm_info->core_mode != XT_MODE_ANY && algorithm_info->core_mode != core_mode) {
			LOG_DEBUG("setting core_mode: 0x%x", algorithm_info->core_mode);
			new_ps = (new_ps & ~XT_PS_RING_MSK) | XT_PS_RING(algorithm_info->core_mode);
			/* save previous core mode */
			algorithm_info->core_mode = core_mode;
		}
	}
	esp108_reg_set(&core_cache->reg_list[XT_REG_IDX_PS], new_ps);
	core_cache->reg_list[XT_REG_IDX_PS].valid = 1;

	retval = target_resume(target, 0, entry_point, 1, 1);

	return retval;
}

int xtensa_wait_algorithm_generic(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t exit_point, int timeout_ms,
	void *arch_info, struct reg_cache *core_cache)
{
	struct xtensa_algorithm *algorithm_info = arch_info;
	int retval = ERROR_OK;
	uint32_t pc;

	/* NOTE: xtensa_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	/* If the target fails to halt due to the breakpoint, force a halt */
	if (retval != ERROR_OK || target->state != TARGET_HALTED) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;
		LOG_ERROR("xtensa_wait_algorithm: not halted %d, pc 0x%x, ps 0x%x", retval,
			esp108_reg_get(&core_cache->reg_list[XT_REG_IDX_PC]), esp108_reg_get(&core_cache->reg_list[XT_REG_IDX_PS]));
		return ERROR_TARGET_TIMEOUT;
	}
	pc = esp108_reg_get(&core_cache->reg_list[XT_REG_IDX_PC]);
	if (exit_point && (pc != exit_point)) {
		LOG_ERROR("failed algorithm halted at 0x%" PRIx32 ", expected 0x%" PRIx32,
			pc,
			exit_point);
		return ERROR_TARGET_TIMEOUT;
	}
	/* Read memory values to mem_params */
	LOG_DEBUG("Read mem params");
	for (int i = 0; i < num_mem_params; i++) {
		LOG_DEBUG("Check mem param @ 0x%x", mem_params[i].address);
		if (mem_params[i].direction != PARAM_OUT) {
			LOG_USER("Read mem param @ 0x%x", mem_params[i].address);
			retval = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	/* Copy core register values to reg_params[] */
	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {
			struct reg *reg = register_get_by_name(core_cache, reg_params[i].reg_name, 0);
			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			if (reg->size != reg_params[i].size) {
				LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			buf_set_u32(reg_params[i].value, 0, 32, esp108_reg_get(reg));
		}
	}

	for (int i = core_cache->num_regs - 1; i >= 0; i--) {
		uint32_t regvalue;
		regvalue = esp108_reg_get(&core_cache->reg_list[i]);
		// LOG_DEBUG("check register %s with value 0x%x -> 0x%8.8" PRIx32,
		// 		core_cache->reg_list[i].name, regvalue, algorithm_info->context[i]);
		if (i == XT_REG_IDX_DEBUGCAUSE) {
			//FIXME: restoring DEBUGCAUSE causes exception when executing corresponding instruction in DIR
			LOG_DEBUG("Skip restoring register %s with value 0x%x -> 0x%8.8" PRIx32,
					core_cache->reg_list[i].name, regvalue, algorithm_info->context[i]);
			continue;
		}
		if (regvalue != algorithm_info->context[i]) {
			LOG_DEBUG("restoring register %s with value 0x%x -> 0x%8.8" PRIx32,
					core_cache->reg_list[i].name, regvalue, algorithm_info->context[i]);
			esp108_reg_set(&core_cache->reg_list[i], algorithm_info->context[i]);
			core_cache->reg_list[i].valid = 1;
		}
	}

	return retval;
}

int xtensa_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	int timeout_ms, void *arch_info)
{
	int retval;

	retval = target->type->start_algorithm(target,
			num_mem_params, mem_params,
			num_reg_params, reg_params,
			entry_point, exit_point,
			arch_info);

	if (retval == ERROR_OK)
		retval = target->type->wait_algorithm(target,
				num_mem_params, mem_params,
				num_reg_params, reg_params,
				exit_point, timeout_ms,
				arch_info);

	return retval;
}

/* Reset ESP32's peripherals.
Postconditions: all peripherals except RTC_CNTL are reset, CPU's PC is undefined, PRO CPU is halted, APP CPU is in reset
How this works:
0. make sure target is halted; if not, try to halt it; if that fails, try to reset it (via OCD) and then halt
1. set CPU initial PC to 0x50000000 (RTC_SLOW_MEM) by clearing RTC_CNTL_{PRO,APP}CPU_STAT_VECTOR_SEL
2. load stub code into RTC_SLOW_MEM; once executed, stub code will disable watchdogs and make CPU spin in an idle loop.
3. trigger SoC reset using RTC_CNTL_SW_SYS_RST bit
4. wait for the OCD to be reset
5. halt the target and wait for it to be halted (at this point CPU is in the idle loop)
6. restore initial PC and the contents of RTC_SLOW_MEM
TODO: some state of RTC_CNTL is not reset during SW_SYS_RST. Need to reset that manually.
*/
int esp32_soc_reset(struct target *target)
{
	LOG_DEBUG("%s %d", __func__, __LINE__);
	int res;

	/* In order to write to peripheral registers, target must be halted first */
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("%s: Target not halted before SoC reset, trying to halt it first", __func__);
		target->type->halt(target);
		res = target_wait_state(target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_DEBUG("%s: Couldn't halt target before SoC reset, trying to do reset-halt", __func__);
			res = target->type->assert_reset(target);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Couldn't halt target before SoC reset! (xtensa_assert_reset returned %d)", __func__, res);
				return res;
			}
			alive_sleep(10);
			target->type->poll(target);
			int reset_halt_save = target->reset_halt;
			target->reset_halt = 1;
			res = target->type->deassert_reset(target);
			target->reset_halt = reset_halt_save;
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)", __func__, res);
				return res;
			}
			alive_sleep(10);
			target->type->poll(target);
			target->type->halt(target);
			res = target_wait_state(target, TARGET_HALTED, 1000);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Couldn't halt target before SoC reset", __func__);
				return res;
			}
		}
	}

	assert(target->state == TARGET_HALTED);

	/* This this the stub code compiled from esp32_cpu_reset_handler.S.
	   To compile it, run:
	       xtensa-esp32-elf-as -o stub.o esp32_cpu_reset_handler.S
	       xtensa-esp32-elf-objcopy -j .text -O binary stub.o stub.bin
	   These steps are not included into OpenOCD build process so that a
	   dependency on xtensa-esp32-elf toolchain is not introduced.
	*/
	const uint32_t esp32_post_reset_code[] = {
		0x00000806, 0x50d83aa1, 0x00000000, 0x3ff480a4, 0x3ff4808c, 0x3ff5f064, 0x3ff5f048, 0x3ff60064,
		0x3ff60048, 0x41fff831, 0x0439fff9, 0x39fffa41, 0xfffa4104, 0xf4310439, 0xfff541ff, 0xf6410439,
		0x410439ff, 0x0439fff7, 0x46007000,
	};

	uint32_t slow_mem_save[sizeof(esp32_post_reset_code) / sizeof(uint32_t)];

	const int RTC_SLOW_MEM_BASE = 0x50000000;
	/* Save contents of RTC_SLOW_MEM which we are about to overwrite */
	res = target->type->read_buffer(target, RTC_SLOW_MEM_BASE, sizeof(slow_mem_save), (uint8_t*)slow_mem_save);
	if (res != ERROR_OK)  {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	/* Write stub code into RTC_SLOW_MEM */
	res = target->type->write_buffer(target, RTC_SLOW_MEM_BASE, sizeof(esp32_post_reset_code), (const uint8_t*)esp32_post_reset_code);
	if (res != ERROR_OK)  {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	const int RTC_CNTL_RESET_STATE_REG = 0x3ff48034;
	const int RTC_CNTL_RESET_STATE_DEF = 0x3000;
	const int RTC_CNTL_CLK_CONF_REG = 0x3ff48070;
	const int RTC_CNTL_CLK_CONF_DEF = 0x2210;
	const int RTC_CNTL_STORE4_REG = 0x3ff480b0;
	const int RTC_CNTL_STORE5_REG = 0x3ff480b4;
	const int RTC_CNTL_OPTIONS0_REG = 0x3ff48000;
	const int RTC_CNTL_OPTIONS0_DEF = 0x1c492000;
	const int RTC_CNTL_SW_SYS_RST = 0x80000000;
	const int DPORT_APPCPU_CTRL_A_REG = 0x3ff0002c;
	const int DPORT_APPCPU_CTRL_B_REG = 0x3ff00030;
	const int DPORT_APPCPU_CLKGATE_EN = 0x1;
	const int DPORT_APPCPU_CTRL_D_REG = 0x3ff00038;

	/* Set a list of registers to these values */
	const uint32_t reg_value_pairs_pre[] = {
		/* Set entry point to RTC_SLOW_MEM */
		RTC_CNTL_RESET_STATE_REG, 0,
		/* Reset SoC clock to XTAL, in case it was running from PLL */
		RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CLK_CONF_DEF,
		/* Reset RTC_CNTL_STORE{4,5}_REG, which are related to clock state */
		RTC_CNTL_STORE4_REG, 0,
		RTC_CNTL_STORE5_REG, 0,
		/* Perform reset */
		RTC_CNTL_OPTIONS0_REG, RTC_CNTL_OPTIONS0_DEF | RTC_CNTL_SW_SYS_RST
	};
	res = xtensa_write_uint32_list(target, reg_value_pairs_pre, sizeof(reg_value_pairs_pre) / 8);
	if (res != ERROR_OK)  {
		LOG_WARNING("%s xtensa_write_uint32_list (reg_value_pairs_pre) err=%d", __func__, res);
		return res;
	}

	/* Wait for SoC to reset */
	int timeout = 100;
	while (target->state != TARGET_RESET && target->state != TARGET_RUNNING && --timeout > 0) {
		alive_sleep(10);
		target->type->poll(target);
	}
	if (timeout == 0) {
		LOG_ERROR("%s: Timed out waiting for CPU to be reset, target->state=%d", __func__, target->state);
		return ERROR_TARGET_TIMEOUT;
	}

	/* Halt the CPU again */
	target->type->halt(target);
	res = target_wait_state(target, TARGET_HALTED, 1000);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Timed out waiting for CPU to be halted after SoC reset", __func__);
		return res;
	}

	const uint32_t reg_value_pairs_post[] = {
		/* Reset entry point back to the reset vector */
		RTC_CNTL_RESET_STATE_REG, RTC_CNTL_RESET_STATE_DEF,
		/* Clear APP CPU boot address */
		DPORT_APPCPU_CTRL_D_REG, 0,
		/* Enable clock to APP CPU */
		DPORT_APPCPU_CTRL_B_REG, DPORT_APPCPU_CLKGATE_EN,
		/* Take APP CPU out of reset */
		DPORT_APPCPU_CTRL_A_REG, 0,
	};
	res = xtensa_write_uint32_list(target, reg_value_pairs_post, sizeof(reg_value_pairs_post) / 8);
	if (res != ERROR_OK)  {
		LOG_WARNING("%s xtensa_write_uint32_list (reg_value_pairs_post) err=%d", __func__, res);
		return res;
	}

	/* Restore the original contents of RTC_SLOW_MEM */
	res = target->type->write_buffer(target, RTC_SLOW_MEM_BASE, sizeof(slow_mem_save), (const uint8_t*)slow_mem_save);
	if (res != ERROR_OK)  {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	/* Clear memory which is used by RTOS layer to get the task count */
	if (target->rtos && target->rtos->type->post_reset_cleanup) {
		res = (*target->rtos->type->post_reset_cleanup)(target);
		if (res != ERROR_OK) {
			LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
		}
	}

	LOG_DEBUG("%s %d", __func__, __LINE__);

	return ERROR_OK;
}


addr_type_t esp108_get_addr_type(uint32_t address)
{
	const uint32_t valid_ranges[] = {
		ESP32_DROM_LOW, ESP32_DROM_HIGH, READONLY,
		ESP32_EXT_RAM_LOW, ESP32_EXT_RAM_HIGH, READWRITE,
		ESP32_DPORT_LOW, ESP32_DPORT_HIGH, READWRITE,
		ESP32_DRAM_LOW, ESP32_DRAM_HIGH, READWRITE,
		ESP32_IRAM00_LOW, ESP32_IRAM00_HIGH, READONLY,
		ESP32_IRAM02_LOW, ESP32_IRAM02_HIGH, READWRITE,
		ESP32_RTC_IRAM_LOW, ESP32_RTC_IRAM_HIGH, READWRITE,
		ESP32_IROM_LOW, ESP32_IROM_HIGH, READONLY,
		ESP32_RTC_DATA_LOW, ESP32_RTC_DATA_HIGH, READWRITE
	};

	const size_t range_count = sizeof(valid_ranges) / sizeof(uint32_t) / 3;

	for (size_t i = 0; i < range_count; ++i) {
		uint32_t low = valid_ranges[3 * i];
		uint32_t high = valid_ranges[3 * i + 1];
		addr_type_t type = valid_ranges[3 * i + 2];
		if (address < low) return INVALID;
		if (address < high) return type;
	}
	return INVALID;
}

//Small helper function to convert the char arrays that result from a jtag
//call to a well-formatted uint32_t.
inline uint32_t intfromchars(uint8_t *c)
{
	return c[0] + (c[1] << 8) + (c[2] << 16) + (c[3] << 24);
}
