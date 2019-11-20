/***************************************************************************
 *   Generic Xtensa target API for OpenOCD                                 *
 *   Copyright (C) 2016-2019 Espressif Systems Ltd.                        *
 *   Derived from esp108.c                                                 *
 *   Author: Angus Gratton gus@projectgus.com                              *
 *   Author: Jeroen Domburg <jeroen@espressif.com>                         *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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

#include "xtensa.h"
#include "xtensa_algorithm.h"
#include "register.h"
#include "time_support.h"

#define XT_WATCHPOINTS_NUM_MAX  2

/* Special register number macro for DDR register.
* this gets used a lot so making a shortcut to it is
* useful.
*/
#define XT_SR_DDR         (xtensa_regs[XT_REG_IDX_DDR].reg_num)

/*Same thing for A3/A4 */
#define XT_REG_A3         (xtensa_regs[XT_REG_IDX_AR3].reg_num)
#define XT_REG_A4         (xtensa_regs[XT_REG_IDX_AR4].reg_num)

#define XT_PC_REG_NUM_BASE          (176)
#define XT_SW_BREAKPOINTS_MAX_NUM   32


const struct xtensa_reg_desc xtensa_regs[XT_NUM_REGS] = {
	{ "pc",             XT_PC_REG_NUM_BASE /*+XT_DEBUGLEVEL*/, XT_REG_SPECIAL, 0 },	/*actually
											 * epc[debuglevel] */
	{ "ar0",                0x00, XT_REG_GENERAL, 0 },
	{ "ar1",                0x01, XT_REG_GENERAL, 0 },
	{ "ar2",                0x02, XT_REG_GENERAL, 0 },
	{ "ar3",                0x03, XT_REG_GENERAL, 0 },
	{ "ar4",                0x04, XT_REG_GENERAL, 0 },
	{ "ar5",                0x05, XT_REG_GENERAL, 0 },
	{ "ar6",                0x06, XT_REG_GENERAL, 0 },
	{ "ar7",                0x07, XT_REG_GENERAL, 0 },
	{ "ar8",                0x08, XT_REG_GENERAL, 0 },
	{ "ar9",                0x09, XT_REG_GENERAL, 0 },
	{ "ar10",               0x0A, XT_REG_GENERAL, 0 },
	{ "ar11",               0x0B, XT_REG_GENERAL, 0 },
	{ "ar12",               0x0C, XT_REG_GENERAL, 0 },
	{ "ar13",               0x0D, XT_REG_GENERAL, 0 },
	{ "ar14",               0x0E, XT_REG_GENERAL, 0 },
	{ "ar15",               0x0F, XT_REG_GENERAL, 0 },
	{ "ar16",               0x10, XT_REG_GENERAL, 0 },
	{ "ar17",               0x11, XT_REG_GENERAL, 0 },
	{ "ar18",               0x12, XT_REG_GENERAL, 0 },
	{ "ar19",               0x13, XT_REG_GENERAL, 0 },
	{ "ar20",               0x14, XT_REG_GENERAL, 0 },
	{ "ar21",               0x15, XT_REG_GENERAL, 0 },
	{ "ar22",               0x16, XT_REG_GENERAL, 0 },
	{ "ar23",               0x17, XT_REG_GENERAL, 0 },
	{ "ar24",               0x18, XT_REG_GENERAL, 0 },
	{ "ar25",               0x19, XT_REG_GENERAL, 0 },
	{ "ar26",               0x1A, XT_REG_GENERAL, 0 },
	{ "ar27",               0x1B, XT_REG_GENERAL, 0 },
	{ "ar28",               0x1C, XT_REG_GENERAL, 0 },
	{ "ar29",               0x1D, XT_REG_GENERAL, 0 },
	{ "ar30",               0x1E, XT_REG_GENERAL, 0 },
	{ "ar31",               0x1F, XT_REG_GENERAL, 0 },
	{ "ar32",               0x20, XT_REG_GENERAL, 0 },
	{ "ar33",               0x21, XT_REG_GENERAL, 0 },
	{ "ar34",               0x22, XT_REG_GENERAL, 0 },
	{ "ar35",               0x23, XT_REG_GENERAL, 0 },
	{ "ar36",               0x24, XT_REG_GENERAL, 0 },
	{ "ar37",               0x25, XT_REG_GENERAL, 0 },
	{ "ar38",               0x26, XT_REG_GENERAL, 0 },
	{ "ar39",               0x27, XT_REG_GENERAL, 0 },
	{ "ar40",               0x28, XT_REG_GENERAL, 0 },
	{ "ar41",               0x29, XT_REG_GENERAL, 0 },
	{ "ar42",               0x2A, XT_REG_GENERAL, 0 },
	{ "ar43",               0x2B, XT_REG_GENERAL, 0 },
	{ "ar44",               0x2C, XT_REG_GENERAL, 0 },
	{ "ar45",               0x2D, XT_REG_GENERAL, 0 },
	{ "ar46",               0x2E, XT_REG_GENERAL, 0 },
	{ "ar47",               0x2F, XT_REG_GENERAL, 0 },
	{ "ar48",               0x30, XT_REG_GENERAL, 0 },
	{ "ar49",               0x31, XT_REG_GENERAL, 0 },
	{ "ar50",               0x32, XT_REG_GENERAL, 0 },
	{ "ar51",               0x33, XT_REG_GENERAL, 0 },
	{ "ar52",               0x34, XT_REG_GENERAL, 0 },
	{ "ar53",               0x35, XT_REG_GENERAL, 0 },
	{ "ar54",               0x36, XT_REG_GENERAL, 0 },
	{ "ar55",               0x37, XT_REG_GENERAL, 0 },
	{ "ar56",               0x38, XT_REG_GENERAL, 0 },
	{ "ar57",               0x39, XT_REG_GENERAL, 0 },
	{ "ar58",               0x3A, XT_REG_GENERAL, 0 },
	{ "ar59",               0x3B, XT_REG_GENERAL, 0 },
	{ "ar60",               0x3C, XT_REG_GENERAL, 0 },
	{ "ar61",               0x3D, XT_REG_GENERAL, 0 },
	{ "ar62",               0x3E, XT_REG_GENERAL, 0 },
	{ "ar63",               0x3F, XT_REG_GENERAL, 0 },
	{ "lbeg",               0x00, XT_REG_SPECIAL, 0 },
	{ "lend",               0x01, XT_REG_SPECIAL, 0 },
	{ "lcount",             0x02, XT_REG_SPECIAL, 0 },
	{ "sar",                0x03, XT_REG_SPECIAL, 0 },
	{ "windowbase",             0x48, XT_REG_SPECIAL, 0 },
	{ "windowstart",                0x49, XT_REG_SPECIAL, 0 },
	{ "configid0",              0xB0, XT_REG_SPECIAL, 0 },
	{ "configid1",              0xD0, XT_REG_SPECIAL, 0 },
	{ "ps",             0xC6, XT_REG_SPECIAL, 0 },	/*actually EPS[debuglevel] */
	{ "threadptr",              0xE7, XT_REG_USER, 0 },
	{ "br",             0x04, XT_REG_SPECIAL, 0 },
	{ "scompare1",              0x0C, XT_REG_SPECIAL, 0 },
	{ "acclo",              0x10, XT_REG_SPECIAL, 0 },
	{ "acchi",              0x11, XT_REG_SPECIAL, 0 },
	{ "m0",             0x20, XT_REG_SPECIAL, 0 },
	{ "m1",             0x21, XT_REG_SPECIAL, 0 },
	{ "m2",             0x22, XT_REG_SPECIAL, 0 },
	{ "m3",             0x23, XT_REG_SPECIAL, 0 },
	{ "expstate",               0xE6, XT_REG_USER, 0 },
	{ "f64r_lo",                0xEA, XT_REG_USER, 0 },
	{ "f64r_hi",                0xEB, XT_REG_USER, 0 },
	{ "f64s",               0xEC, XT_REG_USER, 0 },
	{ "f0",             0x00, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f1",             0x01, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f2",             0x02, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f3",             0x03, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f4",             0x04, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f5",             0x05, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f6",             0x06, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f7",             0x07, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f8",             0x08, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f9",             0x09, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f10",                0x0A, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f11",                0x0B, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f12",                0x0C, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f13",                0x0D, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f14",                0x0E, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f15",                0x0F, XT_REG_FR, XT_REGF_COPROC0 },
	{ "fcr",                0xE8, XT_REG_USER, XT_REGF_COPROC0 },
	{ "fsr",                0xE9, XT_REG_USER, XT_REGF_COPROC0 },
	{ "mmid",               0x59, XT_REG_SPECIAL, XT_REGF_NOREAD },
	{ "ibreakenable",               0x60, XT_REG_SPECIAL, 0 },
	{ "memctl",             0x61, XT_REG_SPECIAL, 0 },
	{ "atomctl",                0x63, XT_REG_SPECIAL, 0 },
	{ "ddr",                0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "ibreaka0",               0x80, XT_REG_SPECIAL, 0 },
	{ "ibreaka1",               0x81, XT_REG_SPECIAL, 0 },
	{ "dbreaka0",               0x90, XT_REG_SPECIAL, 0 },
	{ "dbreaka1",               0x91, XT_REG_SPECIAL, 0 },
	{ "dbreakc0",               0xA0, XT_REG_SPECIAL, 0 },
	{ "dbreakc1",               0xA1, XT_REG_SPECIAL, 0 },
	{ "epc1",               0xB1, XT_REG_SPECIAL, 0 },
	{ "epc2",               0xB2, XT_REG_SPECIAL, 0 },
	{ "epc3",               0xB3, XT_REG_SPECIAL, 0 },
	{ "epc4",               0xB4, XT_REG_SPECIAL, 0 },
	{ "epc5",               0xB5, XT_REG_SPECIAL, 0 },
	{ "epc6",               0xB6, XT_REG_SPECIAL, 0 },
	{ "epc7",               0xB7, XT_REG_SPECIAL, 0 },
	{ "depc",               0xC0, XT_REG_SPECIAL, 0 },
	{ "eps2",               0xC2, XT_REG_SPECIAL, 0 },
	{ "eps3",               0xC3, XT_REG_SPECIAL, 0 },
	{ "eps4",               0xC4, XT_REG_SPECIAL, 0 },
	{ "eps5",               0xC5, XT_REG_SPECIAL, 0 },
	{ "eps6",               0xC6, XT_REG_SPECIAL, 0 },
	{ "eps7",               0xC7, XT_REG_SPECIAL, 0 },
	{ "excsave1",               0xD1, XT_REG_SPECIAL, 0 },
	{ "excsave2",               0xD2, XT_REG_SPECIAL, 0 },
	{ "excsave3",               0xD3, XT_REG_SPECIAL, 0 },
	{ "excsave4",               0xD4, XT_REG_SPECIAL, 0 },
	{ "excsave5",               0xD5, XT_REG_SPECIAL, 0 },
	{ "excsave6",               0xD6, XT_REG_SPECIAL, 0 },
	{ "excsave7",               0xD7, XT_REG_SPECIAL, 0 },
	{ "cpenable",               0xE0, XT_REG_SPECIAL, 0 },
	{ "interrupt",              0xE2, XT_REG_SPECIAL, 0 },
	{ "intset",             0xE2, XT_REG_SPECIAL, XT_REGF_NOREAD },
	{ "intclear",               0xE3, XT_REG_SPECIAL, XT_REGF_NOREAD },
	{ "intenable",              0xE4, XT_REG_SPECIAL, 0 },
	{ "vecbase",                0xE7, XT_REG_SPECIAL, 0 },
	{ "exccause",               0xE8, XT_REG_SPECIAL, 0 },
	{ "debugcause",             0xE9, XT_REG_SPECIAL, 0 },
	{ "ccount",             0xEA, XT_REG_SPECIAL, 0 },
	{ "prid",               0xEB, XT_REG_SPECIAL, 0 },
	{ "icount",             0xEC, XT_REG_SPECIAL, 0 },
	{ "icountlevel",                0xED, XT_REG_SPECIAL, 0 },
	{ "excvaddr",               0xEE, XT_REG_SPECIAL, 0 },
	{ "ccompare0",              0xF0, XT_REG_SPECIAL, 0 },
	{ "ccompare1",              0xF1, XT_REG_SPECIAL, 0 },
	{ "ccompare2",              0xF2, XT_REG_SPECIAL, 0 },
	{ "misc0",              0xF4, XT_REG_SPECIAL, 0 },
	{ "misc1",              0xF5, XT_REG_SPECIAL, 0 },
	{ "misc2",              0xF6, XT_REG_SPECIAL, 0 },
	{ "misc3",              0xF7, XT_REG_SPECIAL, 0 },
	{ "litbase",        0x05, XT_REG_SPECIAL, 0 },
	{ "ptevaddr",       0x53, XT_REG_SPECIAL, 0 },
	{ "rasid",          0x5A, XT_REG_SPECIAL, 0 },
	{ "itlbcfg",        0x5B, XT_REG_SPECIAL, 0 },
	{ "dtlbcfg",        0x5C, XT_REG_SPECIAL, 0 },
	{ "mepc",        0x6A, XT_REG_SPECIAL, 0 },
	{ "meps",        0x6B, XT_REG_SPECIAL, 0 },
	{ "mesave",        0x6C, XT_REG_SPECIAL, 0 },
	{ "mesr",        0x6D, XT_REG_SPECIAL, 0 },
	{ "mecr",        0x6E, XT_REG_SPECIAL, 0 },
	{ "mevaddr",        0x6F, XT_REG_SPECIAL, 0 },
	{ "a0",             XT_REG_IDX_AR0, XT_REG_RELGEN, 0 },	/*WARNING: For these registers,
								 * regnum points to the */
	{ "a1",             XT_REG_IDX_AR1, XT_REG_RELGEN, 0 },	/*index of the corresponding ARx
								 * registers, NOT to */
	{ "a2",             XT_REG_IDX_AR2, XT_REG_RELGEN, 0 },	/*the processor register number! */
	{ "a3",             XT_REG_IDX_AR3, XT_REG_RELGEN, 0 },
	{ "a4",             XT_REG_IDX_AR4, XT_REG_RELGEN, 0 },
	{ "a5",             XT_REG_IDX_AR5, XT_REG_RELGEN, 0 },
	{ "a6",             XT_REG_IDX_AR6, XT_REG_RELGEN, 0 },
	{ "a7",             XT_REG_IDX_AR7, XT_REG_RELGEN, 0 },
	{ "a8",             XT_REG_IDX_AR8, XT_REG_RELGEN, 0 },
	{ "a9",             XT_REG_IDX_AR9, XT_REG_RELGEN, 0 },
	{ "a10",                XT_REG_IDX_AR10, XT_REG_RELGEN, 0 },
	{ "a11",                XT_REG_IDX_AR11, XT_REG_RELGEN, 0 },
	{ "a12",                XT_REG_IDX_AR12, XT_REG_RELGEN, 0 },
	{ "a13",                XT_REG_IDX_AR13, XT_REG_RELGEN, 0 },
	{ "a14",                XT_REG_IDX_AR14, XT_REG_RELGEN, 0 },
	{ "a15",                XT_REG_IDX_AR15, XT_REG_RELGEN, 0 },
};

static inline const struct xtensa_local_mem_region_config *xtensa_memory_region_find(
	const struct xtensa_local_mem_config *mem,
	target_addr_t address)
{
	for (int i = 0; i < mem->count; i++) {
		const struct xtensa_local_mem_region_config *region = &mem->regions[i];
		if (address >= region->base && address < (region->base+region->size))
			return region;
	}
	return NULL;
}

static inline bool xtensa_is_cacheable(const struct xtensa_cache_config *cache,
	const struct xtensa_local_mem_config *mem,
	target_addr_t address)
{
	if (cache->size == 0)
		return false;
	return xtensa_memory_region_find(mem, address) != NULL;
}

static inline bool xtensa_is_icacheable(struct xtensa *xtensa, target_addr_t address)
{
	return xtensa_is_cacheable(&xtensa->core_config->icache, &xtensa->core_config->iram,
		address);
}

static inline bool xtensa_is_dcacheable(struct xtensa *xtensa, target_addr_t address)
{
	return xtensa_is_cacheable(&xtensa->core_config->dcache, &xtensa->core_config->dram,
		address);
}

static int xtensa_get_core_reg(struct reg *reg)
{
	/*We don't need this because we read all registers on halt anyway. */
	struct xtensa *xtensa = (struct xtensa *)reg->arch_info;
	struct target *target = xtensa->target;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	return ERROR_OK;
}

static int xtensa_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct xtensa *xtensa = (struct xtensa *)reg->arch_info;
	struct target *target = xtensa->target;

	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = 1;
	reg->valid = 1;
	return ERROR_OK;
}

static const struct reg_arch_type xtensa_reg_type = {
	.get = xtensa_get_core_reg,
	.set = xtensa_set_core_reg,
};

static inline uint8_t xtensa_insn_size_get(uint8_t *insn)
{
	return insn[0] & 0x8 ? 2 : XT_ISNS_SZ_MAX;
}

/*Convert a register index that's indexed relative to windowbase, to the real address. */
static enum xtensa_reg_id xtensa_windowbase_offset_to_canonical(const enum xtensa_reg_id reg,
	const int windowbase)
{
	int idx;
	if (reg >= XT_REG_IDX_AR0 && reg <= XT_REG_IDX_AR63)
		idx = reg - XT_REG_IDX_AR0;
	else if (reg >= XT_REG_IDX_A0 && reg <= XT_REG_IDX_A15)
		idx = reg - XT_REG_IDX_A0;
	else {
		LOG_ERROR("Error: can't convert register %d to non-windowbased register!\n", reg);
		return -1;
	}
	return ((idx + (windowbase * 4)) & 63) + XT_REG_IDX_AR0;
}

static enum xtensa_reg_id xtensa_canonical_to_windowbase_offset(const enum xtensa_reg_id reg,
	const int windowbase)
{
	return xtensa_windowbase_offset_to_canonical(reg, -windowbase);
}

static bool xtensa_reg_is_readable(int flags, int cpenable)
{
	if (flags & XT_REGF_NOREAD)
		return false;
	if ((flags & XT_REGF_COPROC0) && (cpenable & (1 << 0)) == 0)
		return false;
	return true;
}

static void xtensa_mark_register_dirty(struct xtensa *xtensa, int reg_idx)
{
	struct reg *reg_list = xtensa->core_cache->reg_list;
	reg_list[reg_idx].dirty = 1;
}

static bool xtensa_special_reg_exists(struct xtensa *xtensa, int reg_idx)
{
	/* TODO: array of size XT_NUM_REGS can be used here to map special register ID to
	 * corresponding config option 'enabled' flag */
	if (reg_idx >= XT_REG_IDX_LBEG && reg_idx <= XT_REG_IDX_LCOUNT)
		return xtensa->core_config->loop;
	else if (reg_idx == XT_REG_IDX_BR)
		return xtensa->core_config->boolean;
	else if (reg_idx == XT_REG_IDX_LITBASE)
		return xtensa->core_config->ext_l32r;
	else if (reg_idx == XT_REG_IDX_SCOMPARE1 || reg_idx == XT_REG_IDX_ATOMCTL)
		return xtensa->core_config->cond_store;
	else if (reg_idx >= XT_REG_IDX_ACCLO && reg_idx <= XT_REG_IDX_M3)
		return xtensa->core_config->mac16;
	else if (reg_idx == XT_REG_IDX_WINDOWBASE || reg_idx == XT_REG_IDX_WINDOWSTART)
		return xtensa->core_config->windowed;
	else if (reg_idx >= XT_REG_IDX_PTEVADDR && reg_idx <= XT_REG_IDX_DTLBCFG)
		return xtensa->core_config->mmu.enabled;
	else if (reg_idx == XT_REG_IDX_MMID)
		return xtensa->core_config->trace.enabled;
	else if (reg_idx >= XT_REG_IDX_MEPC && reg_idx <= XT_REG_IDX_MEVADDR)
		return xtensa->core_config->mem_err_check;
	else if (reg_idx == XT_REG_IDX_CPENABLE)
		return xtensa->core_config->coproc;
	else if (reg_idx == XT_REG_IDX_VECBASE)
		return xtensa->core_config->reloc_vec;
	else if (reg_idx == XT_REG_IDX_CCOUNT)
		return xtensa->core_config->tim_irq.enabled;
	else if (reg_idx >= XT_REG_IDX_CCOMPARE0 && reg_idx <= XT_REG_IDX_CCOMPARE2)
		return xtensa->core_config->tim_irq.enabled &&
		       ((reg_idx-XT_REG_IDX_CCOMPARE0) < xtensa->core_config->tim_irq.comp_num);
	else if (reg_idx == XT_REG_IDX_PRID)
		return xtensa->core_config->proc_id;
	else if (reg_idx >= XT_REG_IDX_MISC0 && reg_idx <= XT_REG_IDX_MISC3)
		return ((reg_idx-XT_REG_IDX_MISC0) < xtensa->core_config->miscregs_num);
	return true;
}

static bool xtensa_user_reg_exists(struct xtensa *xtensa, int reg_idx)
{
	if (reg_idx == XT_REG_IDX_THREADPTR)
		return xtensa->core_config->threadptr;
	for (uint8_t i = 0; i < xtensa->core_config->user_regs_num; i++) {
		if (reg_idx == xtensa->core_config->user_regs[i])
			return true;
	}
	return false;
}

static inline bool xtensa_fp_reg_exists(struct xtensa *xtensa, int reg_idx)
{
	return xtensa->core_config->fp_coproc;
}

static inline bool xtensa_regular_reg_exists(struct xtensa *xtensa, int reg_idx)
{
	if (reg_idx >= XT_REG_IDX_AR0 && reg_idx <= XT_REG_IDX_AR63)
		return ((reg_idx-XT_REG_IDX_AR0) < xtensa->core_config->aregs_num);
	return true;
}

static int xtensa_write_dirty_registers(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int i, j;
	int res;
	xtensa_reg_val_t regval, windowbase = 0;
	bool scratch_reg_dirty = false;
	struct reg *reg_list = xtensa->core_cache->reg_list;

	LOG_DEBUG("%s: start", target_name(target));

	/*We need to write the dirty registers in the cache list back to the processor.
	 *Start by writing the SFR/user registers. */
	for (i = 0; i < XT_NUM_REGS; i++) {
		if (reg_list[i].dirty) {
			if (xtensa_regs[i].type == XT_REG_SPECIAL ||
				xtensa_regs[i].type == XT_REG_USER ||
				xtensa_regs[i].type == XT_REG_FR) {
				scratch_reg_dirty = true;
				regval = xtensa_reg_get(target, i);
				LOG_DEBUG("%s: Writing back reg %s val %08X",
					target_name(target),
					xtensa_regs[i].name,
					regval);
				xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, regval);
				xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
				if (xtensa_regs[i].type == XT_REG_USER) {
					if (reg_list[i].exist)
						xtensa_queue_exec_ins(xtensa,
							XT_INS_WUR(xtensa_regs[i].reg_num,
								XT_REG_A3));
				} else if (xtensa_regs[i].type == XT_REG_FR) {
					if (reg_list[i].exist)
						xtensa_queue_exec_ins(xtensa,
							XT_INS_WFR(xtensa_regs[i].reg_num,
								XT_REG_A3));
				} else {/*SFR */
					if (reg_list[i].exist) {
						int reg_num = xtensa_regs[i].reg_num;
						if (reg_num == XT_PC_REG_NUM_BASE) {
							/* reg number of PC for debug interrupt
							 * depends on NDEBUGLEVEL */
							reg_num +=
								xtensa->core_config->debug.
								irq_level;
						}
						xtensa_queue_exec_ins(xtensa,
							XT_INS_WSR(reg_num, XT_REG_A3));
					}
				}
				reg_list[i].dirty = 0;
			}
		}
	}
	if (scratch_reg_dirty)
		xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);

	if (xtensa->core_config->windowed) {
		/*Grab the windowbase, we need it. */
		windowbase = xtensa_reg_get(target, XT_REG_IDX_WINDOWBASE);
		/*Check if there are problems with both the ARx as well as the corresponding Rx
		 * registers set and dirty. */
		/*Warn the user if this happens, not much else we can do... */
		for (i = XT_REG_IDX_A0; i <= XT_REG_IDX_A15; i++) {
			j = xtensa_windowbase_offset_to_canonical(i, windowbase);
			if (reg_list[i].dirty && reg_list[j].dirty) {
				if (memcmp(reg_list[i].value, reg_list[j].value,
						sizeof(xtensa_reg_val_t)) != 0)
					LOG_WARNING(
						"Warning: Both A%d as well as the physical register it points to (AR%d) are dirty and differs in value. Results are undefined!",
						i-XT_REG_IDX_A0,
						j-XT_REG_IDX_AR0);
			}
		}
	}

	/*Write A0-A16 */
	for (i = 0; i < 16; i++) {
		if (reg_list[XT_REG_IDX_A0+i].dirty) {
			regval = xtensa_reg_get(target, XT_REG_IDX_A0+i);
			LOG_DEBUG("%s: Writing back reg %s value %08X, num =%i",
				target_name(target),
				xtensa_regs[XT_REG_IDX_A0 + i].name,
				regval,
				xtensa_regs[XT_REG_IDX_A0 + i].reg_num);
			xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, regval);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, i));
			reg_list[XT_REG_IDX_A0+i].dirty = 0;
		}
	}

	if (xtensa->core_config->windowed) {
		/*Now write AR0-AR63. */
		for (j = 0; j < 64; j += 16) {
			/*Write the 16 registers we can see */
			for (i = 0; i < 16; i++) {
				if (i+j < xtensa->core_config->aregs_num) {
					enum xtensa_reg_id realadr =
						xtensa_windowbase_offset_to_canonical(
						XT_REG_IDX_AR0+i+j,
						windowbase);
					/*Write back any dirty un-windowed registers */
					if (reg_list[realadr].dirty) {
						regval = xtensa_reg_get(target, realadr);
						LOG_DEBUG(
							"%s: Writing back reg %s value %08X, num =%i",
							target_name(target),
							xtensa_regs[realadr].name,
							regval,
							xtensa_regs[realadr].reg_num);
						xtensa_queue_dbg_reg_write(xtensa,
							NARADR_DDR,
							regval);
						xtensa_queue_exec_ins(xtensa,
							XT_INS_RSR(XT_SR_DDR,
								xtensa_regs[XT_REG_IDX_AR0+
									i].reg_num));
						reg_list[realadr].dirty = 0;
					}
				}
			}
			/*Now rotate the window so we'll see the next 16 registers. The final rotate
			 * will wraparound, */
			/*leaving us in the state we were. */
			xtensa_queue_exec_ins(xtensa, XT_INS_ROTW(4));
		}
	}
	res = jtag_execute_queue();
	xtensa_core_status_check(target);

	return res;
}

static inline bool xtensa_is_stopped(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return (xtensa->dbg_mod.core_status.dsr & OCDDSR_STOPPED);
}

int xtensa_examine(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res, cmd;

	LOG_DEBUG("%s coreid=%d", __func__, target->coreid);
	cmd = PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP;
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd);
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd | PWRCTL_JTAGDEBUGUSE);
	xtensa_dm_queue_enable(&xtensa->dbg_mod);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	if (!xtensa_dm_is_online(&xtensa->dbg_mod)) {
		LOG_DEBUG("OCD_ID = %08x", xtensa->dbg_mod.device_id);
		return ERROR_TARGET_FAILURE;
	}
	LOG_DEBUG("OCD_ID = %08x", xtensa->dbg_mod.device_id);
	if (!target_was_examined(target))
		target_set_examined(target);
	return ERROR_OK;
}

int xtensa_wakeup(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int cmd = PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP;

	if (xtensa->reset_asserted)
		cmd |= PWRCTL_CORERESET;
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd);
	/* TODO: can we join this with the write above? */
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd | PWRCTL_JTAGDEBUGUSE);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	return jtag_execute_queue();
}

int xtensa_smpbreak_set(struct target *target, uint32_t set)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = ERROR_OK;
	uint32_t dsr_data = 0x00110000;
	uint32_t clear = set ^
		(OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN|OCDDCR_RUNSTALLINEN|OCDDCR_DEBUGMODEOUTEN);

	xtensa_queue_dbg_reg_write(xtensa, NARADR_DCRSET, set|OCDDCR_ENABLEOCD);
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DCRCLR, clear);
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DSR, dsr_data);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();

	LOG_DEBUG("%s set smpbreak=%x, state=%i", __func__, set, xtensa->target->state);
	return res;
}

static inline xtensa_reg_val_t xtensa_reg_get_value(struct reg *reg)
{
	return *((xtensa_reg_val_t *)reg->value);
}

static inline void xtensa_reg_set_value(struct reg *reg, xtensa_reg_val_t value)
{
	*((xtensa_reg_val_t *)reg->value) = value;
	reg->dirty = 1;
}

int xtensa_core_status_check(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	xtensa_dsr_t dsr;
	int res, needclear = 0;

	xtensa_dm_core_status_read(&xtensa->dbg_mod);
	dsr = xtensa_dm_core_status_get(&xtensa->dbg_mod);
	LOG_DEBUG("%s: DSR (%08X)", target_name(target), dsr);
	if (dsr & OCDDSR_EXECBUSY) {
		if (!xtensa->suppress_dsr_errors)
			LOG_ERROR("%s: DSR (%08X) indicates target still busy!", target_name(
					target), dsr);
		needclear = 1;
	}
	if (dsr & OCDDSR_EXECEXCEPTION) {
		if (!xtensa->suppress_dsr_errors)
			LOG_ERROR(
				"%s: DSR (%08X) indicates DIR instruction generated an exception!",
				target_name(target),
				dsr);
		needclear = 1;
	}
	if (dsr & OCDDSR_EXECOVERRUN) {
		if (!xtensa->suppress_dsr_errors)
			LOG_ERROR("%s: DSR (%08X) indicates DIR instruction generated an overrun!",
				target_name(target),
				dsr);
		needclear = 1;
	}
	if (needclear) {
		res = xtensa_dm_core_status_clear(&xtensa->dbg_mod,
			OCDDSR_EXECEXCEPTION|OCDDSR_EXECOVERRUN);
		if (res != ERROR_OK)
			LOG_ERROR("%s: clearing DSR failed!", target_name(target));
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

xtensa_reg_val_t xtensa_reg_get(struct target *target, enum xtensa_reg_id reg_id)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg = &xtensa->core_cache->reg_list[reg_id];
	return xtensa_reg_get_value(reg);
}

void xtensa_reg_set(struct target *target, enum xtensa_reg_id reg_id, xtensa_reg_val_t value)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg = &xtensa->core_cache->reg_list[reg_id];
	if (xtensa_reg_get_value(reg) == value)
		return;
	xtensa_reg_set_value(reg, value);
}

int xtensa_assert_reset(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;

	LOG_DEBUG("%s[%s] coreid=%i, target_number=%i, begin",
		__func__,
		target_name(target),
		target->coreid,
		target->target_number);
	target->state = TARGET_RESET;
	xtensa_queue_pwr_reg_write(xtensa,
		DMREG_PWRCTL,
		PWRCTL_JTAGDEBUGUSE|PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP|
		PWRCTL_CORERESET);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	xtensa->reset_asserted = true;
	return res;
}

int xtensa_deassert_reset(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;

	LOG_DEBUG("%s coreid=%d halt=%d", __func__, target->coreid, target->reset_halt);
	if (target->reset_halt)
		xtensa_queue_dbg_reg_write(xtensa,
			NARADR_DCRSET,
			OCDDCR_ENABLEOCD|OCDDCR_DEBUGINTERRUPT);
	xtensa_queue_pwr_reg_write(xtensa,
		DMREG_PWRCTL,
		PWRCTL_JTAGDEBUGUSE|PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	target->state = TARGET_RUNNING;
	xtensa->reset_asserted = false;
	return res;
}

int xtensa_fetch_all_regs(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	int i, j, res, cpenable = 0;
	xtensa_reg_val_t regval, windowbase;
	uint8_t regvals[XT_NUM_REGS][sizeof(xtensa_reg_val_t)];
	uint8_t dsrs[XT_NUM_REGS][sizeof(xtensa_dsr_t)];

	LOG_DEBUG("%s: start", target_name(target));

	/*Assume the CPU has just halted. We now want to fill the register cache with all the
	 *register contents GDB needs. For speed, we pipeline all the read operations, execute them
	 *in one go, then sort everything out from the regvals variable. */

	/*Start out with AREGS; we can reach those immediately. Grab them per 16 registers. */
	for (j = 0; j < XT_AREGS_NUM_MAX; j += 16) {
		/*Grab the 16 registers we can see */
		for (i = 0; i < 16; i++) {
			if (i+j < xtensa->core_config->aregs_num) {
				xtensa_queue_exec_ins(xtensa,
					XT_INS_WSR(XT_SR_DDR,
						xtensa_regs[XT_REG_IDX_AR0 + i].reg_num));
				xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR,
					regvals[XT_REG_IDX_AR0 + i + j]);
				xtensa_queue_dbg_reg_read(xtensa, NARADR_DSR,
					dsrs[XT_REG_IDX_AR0 + i + j]);
			}
		}
		if (xtensa->core_config->windowed) {
			/*Now rotate the window so we'll see the next 16 registers. The final rotate
			 * will wraparound, */
			/*leaving us in the state we were. */
			xtensa_queue_exec_ins(xtensa, XT_INS_ROTW(4));
		}
	}
	if (xtensa->core_config->coproc) {
		/*As the very first thing after AREGS, go grab the CPENABLE registers. It indicates
		 * if we can also grab the FP */
		/*(and theoretically other coprocessor) registers, or if this is a bad thing to do.
		 * */
		xtensa_queue_exec_ins(xtensa,
			XT_INS_RSR(xtensa_regs[XT_REG_IDX_CPENABLE].reg_num, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, regvals[XT_REG_IDX_CPENABLE]);
	}
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read ARs (%d)!", res);
		return res;
	}
	xtensa_core_status_check(target);

	if (xtensa->core_config->coproc)
		cpenable = (int)buf_get_u32(regvals[XT_REG_IDX_CPENABLE], 0, 32);
	/*We're now free to use any of A0-A15 as scratch registers
	 *Grab the SFRs and user registers first. We use A3 as a scratch register. */
	for (i = 0; i < XT_NUM_REGS; i++) {
		if (xtensa_reg_is_readable(xtensa_regs[i].flags, cpenable) && reg_list[i].exist &&
			(xtensa_regs[i].type == XT_REG_SPECIAL ||
				xtensa_regs[i].type == XT_REG_USER || xtensa_regs[i].type ==
				XT_REG_FR)) {
			if (xtensa_regs[i].type == XT_REG_USER)
				xtensa_queue_exec_ins(xtensa,
					XT_INS_RUR(xtensa_regs[i].reg_num, XT_REG_A3));
			else if (xtensa_regs[i].type == XT_REG_FR)
				xtensa_queue_exec_ins(xtensa,
					XT_INS_RFR(xtensa_regs[i].reg_num, XT_REG_A3));
			else {	/*SFR */
				int reg_num = xtensa_regs[i].reg_num;
				if (reg_num == XT_PC_REG_NUM_BASE) {
					/* reg number of PC for debug interrupt depends on
					 * NDEBUGLEVEL */
					reg_num += xtensa->core_config->debug.irq_level;
				}
				xtensa_queue_exec_ins(xtensa, XT_INS_RSR(reg_num, XT_REG_A3));
			}
			xtensa_queue_exec_ins(xtensa, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, regvals[i]);
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DSR, dsrs[i]);
		}
	}
	/*Ok, send the whole mess to the CPU. */
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to fetch AR regs!");
		return res;
	}
	xtensa_core_status_check(target);

	/*DSR checking: follows order in which registers are requested. */
	for (i = 0; i < XT_NUM_REGS; i++) {
		if (xtensa_reg_is_readable(xtensa_regs[i].flags, cpenable) && reg_list[i].exist &&
			(xtensa_regs[i].type == XT_REG_SPECIAL ||
				xtensa_regs[i].type == XT_REG_USER || xtensa_regs[i].type ==
				XT_REG_FR)) {
			if (buf_get_u32(dsrs[i], 0, 32) & OCDDSR_EXECEXCEPTION) {
				LOG_ERROR("Exception reading %s!", xtensa_regs[i].name);
				return ERROR_FAIL;
			}
		}
	}

	if (xtensa->core_config->windowed) {
		/*We need the windowbase to decode the general addresses. */
		windowbase = buf_get_u32(regvals[XT_REG_IDX_WINDOWBASE], 0, 32);
		/*Decode the result and update the cache. */
		for (i = 0; i < XT_NUM_REGS; i++) {
			if (xtensa_reg_is_readable(xtensa_regs[i].flags,
					cpenable) && reg_list[i].exist) {
				if (xtensa_regs[i].type == XT_REG_GENERAL) {
					/*The 64-value general register set is read from
					 * (windowbase) on down. We need */
					/*to get the real register address by subtracting windowbase
					 * and wrapping around. */
					int realadr = xtensa_canonical_to_windowbase_offset(i,
						windowbase);
					regval = buf_get_u32(regvals[realadr], 0, 32);
					/*              LOG_INFO("mapping: %s -> %s (windowbase off
					 * %d)\n",xtensa_regs[i].name, xtensa_regs[realadr].name,
					 * windowbase*4); */
				} else if (xtensa_regs[i].type == XT_REG_RELGEN) {
					regval =
						buf_get_u32(regvals[xtensa_regs[i].reg_num], 0, 32);
					/* LOG_DEBUG("%s = 0x%x",xtensa_regs[i].name, regval); */
				} else {
					regval = buf_get_u32(regvals[i], 0, 32);
					/*              LOG_INFO("Register %s: 0x%X",
					 * xtensa_regs[i].name, regval); */
				}
				xtensa_reg_set(target, i, regval);
				reg_list[i].valid = 1;
				reg_list[i].dirty = 0;	/*always do this _after_ xtensa_reg_set! */
				/*LOG_DEBUG("%s Register %s: 0x%X", xtensa->cmd_name,
				 * reg_list[c][i].name, regval); */
			} else {
				reg_list[i].valid = 0;
				/*LOG_DEBUG("Register NOT READBLE %s: 0x%X", reg_list[c][i].name,
				 * 0); */
			}
		}
	}
	/*We have used A3 as a scratch register and we will need to write that back. */
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);

	return ERROR_OK;
}

int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int i, k;

	if (reg_class == REG_CLASS_ALL)
		*reg_list_size = xtensa->regs_num;
	else
		*reg_list_size = xtensa->core_config->gdb_regs_num;

	LOG_DEBUG("reg_class=%i, num_regs=%d", (int)reg_class, *reg_list_size);

	*reg_list = malloc((*reg_list_size)*sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	for (i = 0, k = 0; i < (int)xtensa->core_cache->num_regs && k < *reg_list_size; i++) {
		if (xtensa->core_cache->reg_list[i].exist)
			(*reg_list)[k++] = &xtensa->core_cache->reg_list[i];
	}

	return ERROR_OK;
}

int xtensa_mmu_is_enabled(struct target *target, int *enabled)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	*enabled = xtensa->core_config->mmu.itlb_entries_count > 0 ||
		xtensa->core_config->mmu.dtlb_entries_count > 0;
	return ERROR_OK;
}

int xtensa_halt(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;

	LOG_DEBUG("%s, target: %s", __func__, target_name(target));
	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("%s: target was already halted", target_name(target));
		return ERROR_OK;
	}
	/* First we have to read dsr and check if the target stopped */
	res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to read core status!", target_name(target));
		return res;
	}
	if (!(xtensa_dm_core_status_get(&xtensa->dbg_mod) & OCDDSR_STOPPED)) {
		xtensa_queue_dbg_reg_write(xtensa,
			NARADR_DCRSET,
			OCDDCR_ENABLEOCD|OCDDCR_DEBUGINTERRUPT);
		xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
		res = jtag_execute_queue();
		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to set OCDDCR_DEBUGINTERRUPT. Can't halt.",
				target_name(target));
			return res;
		}
	}

	return ERROR_OK;
}

int xtensa_prepare_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = ERROR_OK;
	size_t slot;
	xtensa_reg_val_t bpena = 0;

	LOG_DEBUG(
		"%s: current=%d address=" TARGET_ADDR_FMT
		", handle_breakpoints=%i, debug_execution=%i)",
		target_name(target),
		current,
		address,
		handle_breakpoints,
		debug_execution);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", target_name(target), __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (address && !current)
		xtensa_reg_set(target, XT_REG_IDX_PC, address);
	else {
		xtensa_reg_val_t cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
		if (cause & DEBUGCAUSE_DB) {
			/*We stopped due to a watchpoint. We can't just resume executing the
			 * instruction again because */
			/*that would trigger the watchpoint again. To fix this, we single-step,
			 * which ignores watchpoints. */
			xtensa_do_step(target, current, address, handle_breakpoints);
		}
		if (cause & (DEBUGCAUSE_BI|DEBUGCAUSE_BN)) {
			/*We stopped due to a break instruction. We can't just resume executing the
			 * instruction again because */
			/*that would trigger the breake again. To fix this, we single-step, which
			 * ignores break. */
			xtensa_do_step(target, current, address, handle_breakpoints);
		}
	}

	/*Write back hw breakpoints. Current FreeRTOS SMP code can set a hw breakpoint on an
	 *exception; we need to clear that and return to the breakpoints gdb has set on resume. */
	for (slot = 0; slot < xtensa->core_config->debug.ibreaks_num; slot++) {
		if (xtensa->hw_brps[slot] != NULL) {
			/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
			xtensa_reg_set(target,
				XT_REG_IDX_IBREAKA0 + slot,
				xtensa->hw_brps[slot]->address);
			bpena |= 1 << slot;
		}
	}
	xtensa_reg_set(target, XT_REG_IDX_IBREAKENABLE, bpena);

	/* Here we write all registers to the targets */
	res = xtensa_write_dirty_registers(target);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to write back register cache.", target_name(target));
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int xtensa_do_resume(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_DEBUG("%s: start", target_name(target));

	xtensa_queue_exec_ins(xtensa, XT_INS_RFDO);
	int res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to exec RFDO %d!",  target_name(target), res);
		return res;
	}
	xtensa_core_status_check(target);
	return ERROR_OK;
}

int xtensa_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	LOG_DEBUG("%s:", target_name(target));

	int res = xtensa_prepare_resume(target,
		current,
		address,
		handle_breakpoints,
		debug_execution);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to prepare for resume!", target_name(target));
		return res;
	}
	res = xtensa_do_resume(target);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to resume!", target_name(target));
		return res;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;
	int res1 = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	if (res1 != ERROR_OK)
		res = res1;
	return res;
}

static bool xtensa_pc_in_winexc(struct target *target, target_addr_t pc)
{
	uint8_t insn_buf[sizeof(xtensa_insn_t)];
	xtensa_insn_t insn;

	int err = xtensa_read_buffer(target, pc, sizeof(insn_buf), insn_buf);
	if (err != ERROR_OK)
		return false;
	insn = buf_get_u32(insn_buf, 0, 24);

	xtensa_insn_t masked = insn & XT_INS_L32E_S32E_MASK;
	if (masked == XT_INS_L32E(0, 0, 0) || masked == XT_INS_S32E(0, 0, 0))
		return true;

	masked = insn & XT_INS_RFWO_RFWU_MASK;
	if (masked == XT_INS_RFWO || masked == XT_INS_RFWU)
		return true;

	return false;
}

int xtensa_do_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res, slot;
	const uint32_t icount_val = -2;	/* ICOUNT value to load for 1 step */
	xtensa_reg_val_t dbreakc[XT_WATCHPOINTS_NUM_MAX];
	xtensa_reg_val_t icountlvl, cause;
	xtensa_reg_val_t oldps, newps, oldpc, cur_pc;

	LOG_DEBUG("%s: current=%d, address="TARGET_ADDR_FMT ", handle_breakpoints=%i",
		target_name(target), current, address, handle_breakpoints);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	if (xtensa->core_config->debug.icount_sz != 32) {
		LOG_WARNING("%s: stepping for ICOUNT less then 32 bits is not implemented!",
			target_name(target));
		return ERROR_FAIL;
	}

	/*Save old ps/pc */
	oldps = xtensa_reg_get(target, XT_REG_IDX_PS);
	oldpc = xtensa_reg_get(target, XT_REG_IDX_PC);

	cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
	LOG_DEBUG("%s: oldps=%x, oldpc=%x dbg_cause=%x exc_cause=%x",
		target_name(target),
		oldps,
		oldpc,
		cause,
		xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE));
	if (handle_breakpoints && (cause & (DEBUGCAUSE_BI|DEBUGCAUSE_BN))) {
		/* handle hard-coded SW breakpoints (e.g. syscalls) */
		LOG_DEBUG("%s: Increment PC to pass break instruction...", target_name(target));
		xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);	/*so we don't recurse into
									 * the same routine */
		xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = 0;
		/* pretend that we have stepped */
		if (cause & DEBUGCAUSE_BI)
			xtensa_reg_set(target, XT_REG_IDX_PC, oldpc + 3);	/* PC = PC+3 */
		else
			xtensa_reg_set(target, XT_REG_IDX_PC, oldpc + 2);	/* PC = PC+2 */
		return ERROR_OK;
	}

	if (xtensa->stepping_isr_mode == XT_STEPPING_ISR_OFF) {
		if (!xtensa->core_config->high_irq.enabled) {
			LOG_WARNING(
				"%s: disabling IRQs while stepping is not implemented w/o high prio IRQs option!",
				target_name(target));
			return ERROR_FAIL;
		}
		/* Increasing the interrupt level to avoid taking interrupts while stepping. */
		xtensa_reg_val_t temp_ps = (oldps & ~0xF) |
			xtensa->core_config->high_irq.excm_level;
		xtensa_reg_set(target, XT_REG_IDX_PS, temp_ps);
		/* Now we have to set up max posssible interrupt level (ilevel + 1) */
		icountlvl = xtensa->core_config->high_irq.excm_level + 1;
	} else
		icountlvl = (oldps & 0xF) + 1;

	if (cause & DEBUGCAUSE_DB) {
		/*We stopped due to a watchpoint. We can't just resume executing the instruction
		 * again because */
		/*that would trigger the watchpoint again. To fix this, we remove watchpoints,
		 * single-step and */
		/*re-enable the watchpoint. */
		LOG_DEBUG(
			"%s: Single-stepping to get past instruction that triggered the watchpoint...",
			target_name(target));
		xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);	/*so we don't recurse into
									 * the same routine */
		xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = 0;
		/*Save all DBREAKCx registers and set to 0 to disable watchpoints */
		for (slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++) {
			dbreakc[slot] = xtensa_reg_get(target, XT_REG_IDX_DBREAKC0+slot);
			xtensa_reg_set(target, XT_REG_IDX_DBREAKC0+slot, 0);
		}
	}

	if (!handle_breakpoints && (cause & (DEBUGCAUSE_BI|DEBUGCAUSE_BN))) {
		/* handle normal SW breakpoint */
		xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);	/*so we don't recurse into
									 * the same routine */
		xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = 0;
	}
	do {
		xtensa_reg_set(target, XT_REG_IDX_ICOUNTLEVEL, icountlvl);
		xtensa_reg_set(target, XT_REG_IDX_ICOUNT, icount_val);

		/* Now ICOUNT is set, we can resume as if we were going to run */
		res = xtensa_prepare_resume(target, current, address, 0, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to prepare resume for single step",
				target_name(target));
			return res;
		}
		res = xtensa_do_resume(target);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to resume after setting up single step",
				target_name(target));
			return res;
		}

		/* Wait for stepping to complete */
		int64_t start = timeval_ms();
		while (timeval_ms() < start+500) {
			/*Do not use target_poll here, it also triggers other things... just
			 * manually read the DSR until stepping */
			/*is complete. */
			usleep(50000);
			res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Failed to read core status!", target_name(target));
				return res;
			}
			if (xtensa_is_stopped(target))
				break;
			else
				usleep(50000);
		}
		LOG_DEBUG("%s: Finish stepping. dsr=0x%08x",
			target_name(target),
			xtensa_dm_core_status_get(&xtensa->dbg_mod));
		if (!xtensa_is_stopped(target)) {
			LOG_WARNING(
				"%s: Timed out waiting for target to finish stepping. dsr=0x%08x",
				target_name(target),
				xtensa_dm_core_status_get(&xtensa->dbg_mod));
			target->debug_reason = DBG_REASON_NOTHALTED;
			target->state = TARGET_RUNNING;
			return ERROR_FAIL;
		}
		target->debug_reason = DBG_REASON_SINGLESTEP;
		target->state = TARGET_HALTED;

		xtensa_fetch_all_regs(target);

		cur_pc = xtensa_reg_get(target, XT_REG_IDX_PC);

		LOG_DEBUG("%s: cur_ps=%x, cur_pc=%x dbg_cause=%x exc_cause=%x",
			target_name(target), xtensa_reg_get(target, XT_REG_IDX_PS), cur_pc,
			xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE),
			xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE));

		if (xtensa->core_config->windowed &&
			xtensa->stepping_isr_mode == XT_STEPPING_ISR_OFF &&
			xtensa_pc_in_winexc(target, cur_pc)) {
			/* isrmask = on, need to step out of the window exception handler */
			LOG_DEBUG("Stepping out of window exception, PC=%X", cur_pc);
			oldpc = cur_pc;
			address = oldpc + 3;
			continue;
		}

		if (oldpc == cur_pc)
			LOG_WARNING("%s: Stepping doesn't seem to change PC! dsr=0x%08x",
				target_name(target),
				xtensa_dm_core_status_get(&xtensa->dbg_mod));
		else
			LOG_DEBUG("Stepped from %X to %X", oldpc, cur_pc);
		break;
	} while (true);
	LOG_DEBUG("Done stepping, PC=%X", cur_pc);

	if (cause & DEBUGCAUSE_DB) {
		LOG_DEBUG("%s: ...Done, re-installing watchpoints.", target_name(target));
		/*Restore the DBREAKCx registers */
		for (slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++)
			xtensa_reg_set(target, XT_REG_IDX_DBREAKC0+slot, dbreakc[slot]);
	}

	/*Restore int level */
	/*ToDo: Theoretically, this can mess up stepping over an instruction that modifies
	 * ps.intlevel */
	/*by itself. Hmmm. ToDo: Look into this. */
	if (xtensa->stepping_isr_mode == XT_STEPPING_ISR_OFF) {
		newps = xtensa_reg_get(target, XT_REG_IDX_PS);
		newps = (newps & ~0xF) | (oldps & 0xf);
		xtensa_reg_set(target, XT_REG_IDX_PS, newps);
	}

	/* write ICOUNTLEVEL back to zero */
	xtensa_reg_set(target, XT_REG_IDX_ICOUNTLEVEL, 0);
	/* TODO: can we skip writing dirty registers and re-fetching them? */
	res = xtensa_write_dirty_registers(target);
	xtensa_fetch_all_regs(target);
	return res;
}

int xtensa_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	int retval = xtensa_do_step(target, current, address, handle_breakpoints);
	if (retval != ERROR_OK)
		return retval;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return retval;
}

static inline bool xtensa_memory_op_validate(struct xtensa *xtensa,
	target_addr_t address,
	int access)
{
	const struct xtensa_local_mem_region_config *mem_region = xtensa_memory_region_find(
		&xtensa->core_config->irom,
		address);
	if (mem_region)
		return (mem_region->access & access) == access;
	mem_region = xtensa_memory_region_find(&xtensa->core_config->drom, address);
	if (mem_region)
		return (mem_region->access & access) == access;
	mem_region = xtensa_memory_region_find(&xtensa->core_config->iram, address);
	if (mem_region)
		return (mem_region->access & access) == access;
	mem_region = xtensa_memory_region_find(&xtensa->core_config->dram, address);
	if (mem_region)
		return (mem_region->access & access) == access;
	mem_region = xtensa_memory_region_find(&xtensa->core_config->uram, address);
	if (mem_region)
		return (mem_region->access & access) == access;
	return true;
}

int xtensa_read_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	/*We are going to read memory in 32-bit increments. This may not be what the calling
	 * function expects, so we may need to allocate a temp buffer and read into that first. */
	target_addr_t addrstart_al = (address) & ~3;
	target_addr_t addrend_al = (address + (size*count) + 3) & ~3;
	target_addr_t adr = addrstart_al;
	int i = 0, res;
	uint8_t *albuff;

/*  LOG_INFO("%s: %s: reading %d bytes from addr %08X", target_name(target), __FUNCTION__,
 * size*count, address); */
/*  LOG_INFO("Converted to aligned addresses: read from %08X to %08X", addrstart_al, addrend_al); */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!xtensa_memory_op_validate(xtensa, address,
			XT_MEM_ACCESS_READ) && !xtensa->permissive_mode) {
		LOG_DEBUG("address "TARGET_ADDR_FMT " not readable", address);
		return ERROR_FAIL;
	}

	if (addrstart_al == address && addrend_al == address + (size*count))
		albuff = buffer;
	else {
		albuff = malloc(addrend_al-addrstart_al);
		if (!albuff) {
			LOG_ERROR("%s: Out of memory allocating %ld bytes!", __FUNCTION__,
				(long)(addrend_al-addrstart_al));
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	/*We're going to use A3 here */
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);
	/*Write start address to A3 */
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrstart_al);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	/*Now we can safely read data from addrstart_al up to addrend_al into albuff */
	while (adr != addrend_al) {
		xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, &albuff[i]);
		adr += sizeof(uint32_t);
		i += sizeof(uint32_t);
	}
	res = jtag_execute_queue();
	if (res == ERROR_OK)
		res = xtensa_core_status_check(target);
	if (res != ERROR_OK)
		LOG_WARNING("%s: Failed reading %d bytes at address "TARGET_ADDR_FMT,
			target_name(target), count*size, address);

	if (albuff != buffer) {
		memcpy(buffer, albuff + (address & 3), (size*count));
		free(albuff);
	}

	return res;
}

int xtensa_read_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	uint8_t *buffer)
{
	/*xtensa_read_memory can also read unaligned stuff. Just pass through to that routine. */
	return xtensa_read_memory(target, address, 1, count, buffer);
}

int xtensa_write_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer)
{
	/*This memory write function can get thrown nigh everything into it, from
	 *aligned uint32 writes to unaligned uint8ths. The Xtensa memory doesn't always
	 *accept anything but aligned uint32 writes, though. That is why we convert
	 *everything into that. */
	struct xtensa *xtensa = target_to_xtensa(target);
	target_addr_t addrstart_al = (address) & ~3;
	target_addr_t addrend_al = (address + (size*count) + 3) & ~3;
	target_addr_t adr = addrstart_al;
	int i = 0, res;
	uint8_t *albuff;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!xtensa_memory_op_validate(xtensa, address,
			XT_MEM_ACCESS_WRITE) && !xtensa->permissive_mode) {
		LOG_DEBUG("address "TARGET_ADDR_FMT " not writable", address);
		return ERROR_FAIL;
	}

/*  LOG_INFO("%s: %s: writing %d bytes to addr %08X", target_name(target), __FUNCTION__, size*count,
 * address); */
/*  LOG_INFO("al start %x al end %x", addrstart_al, addrend_al); */

	if ((size == 0) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	/*Allocate a temporary buffer to put the aligned bytes in, if needed. */
	if (addrstart_al == address && addrend_al == address + (size*count)) {
		/*We discard the const here because albuff can also be non-const */
		albuff = (uint8_t *)buffer;
	} else {
		albuff = malloc(addrend_al-addrstart_al);
		if (!albuff) {
			LOG_ERROR("%s: Out of memory allocating %ld bytes!", __FUNCTION__,
				(long)(addrend_al-addrstart_al));
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	/*We're going to use A3 here */
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);

	/*If we're using a temp aligned buffer, we need to fill the head and/or tail bit of it. */
	if (albuff != buffer) {
		/*See if we need to read the first and/or last word. */
		if (address & 3) {
			xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrstart_al);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(XT_REG_A3));
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, &albuff[0]);
		}
		if ((address + (size*count)) & 3) {
			xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrend_al-4);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(XT_REG_A3));
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR,
				&albuff[addrend_al-addrstart_al-4]);
		}
		/*Grab bytes */
		res = jtag_execute_queue();
		xtensa_core_status_check(target);
		/*Copy data to be written into the aligned buffer */
		memcpy(&albuff[address & 3], buffer, size*count);
		/*Now we can write albuff in aligned uint32s. */
	}

	/*Write start address to A3 */
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrstart_al);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	/*Write the aligned buffer */
	while (adr != addrend_al) {
		xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, buf_get_u32(&albuff[i], 0, 32));
		xtensa_queue_exec_ins(xtensa, XT_INS_SDDR32P(XT_REG_A3));
		adr += 4;
		i += 4;
	}
	res = jtag_execute_queue();
	if (res == ERROR_OK)
		res = xtensa_core_status_check(target);
	if (res != ERROR_OK)
		LOG_WARNING("%s: Failed writing %d bytes at address "TARGET_ADDR_FMT,
			target_name(target), count*size, address);
	if (albuff != buffer)
		free(albuff);

	if (xtensa_is_icacheable(xtensa, address)) {
		/* NB: if we were supporting the ICACHE option, we would need
		 * to invalidate it here */
	}
	if (xtensa_is_dcacheable(xtensa, address)) {
		/* NB: if we were supporting the DCACHE option, we would need
		 * to invalidate it here */
	}

	return res;
}

int xtensa_write_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	const uint8_t *buffer)
{
	/*xtensa_write_memory can handle everything. Just pass on to that. */
	return xtensa_write_memory(target, address, 1, count, buffer);
}

int xtensa_checksum_memory(struct target *target,
	target_addr_t address,
	uint32_t count,
	uint32_t *checksum)
{
	LOG_WARNING("not implemented yet");
	return ERROR_FAIL;
}

/* do some general work upon poll */
void xtensa_on_poll(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;

	if (xtensa->trace_active) {
		/*Detect if tracing was active but has stopped. */
		struct xtensa_trace_status trace_status;
		res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
		if (res == ERROR_OK) {
			if (!(trace_status.stat & TRAXSTAT_TRACT)) {
				LOG_INFO("Detected end of trace.");
				if (trace_status.stat & TRAXSTAT_PCMTG)
					LOG_INFO("%s: Trace stop triggered by PC match",
						target_name(target));
				if (trace_status.stat &TRAXSTAT_PTITG)
					LOG_INFO(
						"%s: Trace stop triggered by Processor Trigger Input",
						target_name(target));
				if (trace_status.stat & TRAXSTAT_CTITG)
					LOG_INFO("%s: Trace stop triggered by Cross-trigger Input",
						target_name(target));
				xtensa->trace_active = false;
			}
		}
	}
}

int xtensa_poll(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	bool need_resume = false;

	int res = xtensa_dm_power_status_read(&xtensa->dbg_mod,
		PWRSTAT_DEBUGWASRESET|PWRSTAT_COREWASRESET);
	if (res != ERROR_OK)
		return res;

	if (xtensa_dm_tap_was_reset(&xtensa->dbg_mod))
		LOG_INFO("%s: Debug controller %d was reset.", target_name(target), target->coreid);
	if (xtensa_dm_core_was_reset(&xtensa->dbg_mod)) {
		LOG_INFO("%s: Core %d was reset.", target_name(target), target->coreid);
		if (xtensa->chip_ops != NULL && xtensa->chip_ops->on_reset != NULL)
			xtensa->chip_ops->on_reset(target);
	}
	xtensa_dm_power_status_cache(&xtensa->dbg_mod);
	/*Enable JTAG, set reset if needed */
	res = xtensa_wakeup(target);
	if (res != ERROR_OK)
		return res;

	res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;
	/*LOG_DEBUG("%s: 0x%x %d", target_name(target), xtensa->dbg_mod.core_status.dsr,
	 * xtensa->dbg_mod.core_status.dsr & OCDDSR_STOPPED); */
	if (xtensa_dm_power_status_get(&xtensa->dbg_mod) & PWRSTAT_COREWASRESET)
		target->state = TARGET_RESET;
	else if (xtensa->dbg_mod.core_status.dsr & OCDDSR_STOPPED) {
		if (target->state != TARGET_HALTED) {
			enum target_state oldstate = target->state;
			target->state = TARGET_HALTED;
			/*Examine why the target has been halted */
			target->debug_reason = DBG_REASON_DBGRQ;
			xtensa_fetch_all_regs(target);
			/* When setting debug reason DEBUGCAUSE events have the followuing
			 * priorites: watchpoint == breakpoint > single step > debug interrupt. */
			/* Watchpoint and breakpoint events at the same time results in special
			 * debug reason: DBG_REASON_WPTANDBKPT. */
			xtensa_reg_val_t halt_cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
			if (halt_cause & DEBUGCAUSE_IC)
				target->debug_reason = DBG_REASON_SINGLESTEP;
			if (halt_cause & (DEBUGCAUSE_IB | DEBUGCAUSE_BN | DEBUGCAUSE_BI)) {
				if (halt_cause & DEBUGCAUSE_DB)
					target->debug_reason = DBG_REASON_WPTANDBKPT;
				else
					target->debug_reason = DBG_REASON_BREAKPOINT;
			} else if (halt_cause & DEBUGCAUSE_DB)
				target->debug_reason = DBG_REASON_WATCHPOINT;
			LOG_DEBUG("%s: Target halted, pc=0x%08X, debug_reason=%08x, oldstate=%08x",
				target_name(target),
				xtensa_reg_get(target, XT_REG_IDX_PC),
				target->debug_reason,
				oldstate);
			LOG_DEBUG("%s: Halt reason=0x%08X, exc_cause=%d, dsr=0x%08x",
				target_name(target),
				halt_cause,
				xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE),
				xtensa->dbg_mod.core_status.dsr);
			LOG_INFO("%s: Target halted, PC=0x%08X, debug_reason=%08x",
				target_name(target),
				xtensa_reg_get(target, XT_REG_IDX_PC), target->debug_reason);
			xtensa_dm_core_status_clear(&xtensa->dbg_mod,
				OCDDSR_DEBUGPENDBREAK|OCDDSR_DEBUGINTBREAK|OCDDSR_DEBUGPENDHOST|
				OCDDSR_DEBUGINTHOST);

			/*Call any event callbacks that are applicable */
			if (oldstate == TARGET_DEBUG_RUNNING)
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			else {
				need_resume = xtensa->chip_ops != NULL &&
					xtensa->chip_ops->on_halt !=
					NULL ? xtensa->chip_ops->on_halt(target) : false;
				/* in case of semihosting call we will resume automatically a bit
				 * later, so do not confuse GDB */
				if (!need_resume)
					target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
		}
	} else {
		target->debug_reason = DBG_REASON_NOTHALTED;
		if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING) {
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}
	if (xtensa->chip_ops != NULL && xtensa->chip_ops->on_poll != NULL)
		xtensa->chip_ops->on_poll(target);

	if (need_resume) {
		res = target_resume(target, 1, 0, 1, 0);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to resume target upon core request!");
	}

	return ERROR_OK;
}

static int xtensa_sw_breakpoint_add(struct target *target,
	struct breakpoint *breakpoint,
	struct xtensa_sw_breakpoint *sw_bp)
{
	union {
		uint32_t d32;
		uint8_t d8[4];
	} break_insn;

	int ret = target_read_buffer(target, breakpoint->address, XT_ISNS_SZ_MAX, break_insn.d8);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to read original instruction (%d)!", target_name(target),
			ret);
		return ret;
	}
	sw_bp->insn_sz = xtensa_insn_size_get(break_insn.d8);
	memcpy(sw_bp->insn, break_insn.d8, sw_bp->insn_sz);
	sw_bp->oocd_bp = breakpoint;

	break_insn.d32 = sw_bp->insn_sz == XT_ISNS_SZ_MAX ? XT_INS_BREAK(0, 0) : XT_INS_BREAKN(0);

	ret = target_write_buffer(target, breakpoint->address, sw_bp->insn_sz, break_insn.d8);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to write breakpoint instruction (%d)!",
			target_name(target),
			ret);
		return ret;
	}

	return ERROR_OK;
}

static int xtensa_sw_breakpoint_remove(struct target *target, struct xtensa_sw_breakpoint *sw_bp)
{
	int ret = target_write_buffer(target, sw_bp->oocd_bp->address, sw_bp->insn_sz, sw_bp->insn);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to read insn (%d)!", target_name(target), ret);
		return ret;
	}
	sw_bp->oocd_bp = NULL;
	return ERROR_OK;
}

int xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint32_t slot;

	if (breakpoint->type == BKPT_SOFT) {
		for (slot = 0; slot < XT_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if (xtensa->sw_brps[slot].oocd_bp == NULL ||
				xtensa->sw_brps[slot].oocd_bp == breakpoint)
				break;
		}
		if (slot == XT_SW_BREAKPOINTS_MAX_NUM) {
			LOG_WARNING("%s: No free slots to add SW breakpoint!", target_name(target));
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		int ret = xtensa_sw_breakpoint_add(target, breakpoint, &xtensa->sw_brps[slot]);
		if (ret != ERROR_OK) {
			LOG_ERROR("%s: Failed to add SW breakpoint!", target_name(target));
			return ret;
		}
		LOG_DEBUG("%s: placed SW breakpoint %u @ " TARGET_ADDR_FMT,
			target_name(target),
			slot,
			breakpoint->address);
		return ERROR_OK;
	}

	for (slot = 0; slot < xtensa->core_config->debug.ibreaks_num; slot++) {
		if (xtensa->hw_brps[slot] == NULL || xtensa->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.ibreaks_num) {
		LOG_WARNING("%s: No free slots to add HW breakpoint!", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	xtensa->hw_brps[slot] = breakpoint;
	/*We will actually write the breakpoints when we resume the target. */
	LOG_DEBUG("%s: placed HW breakpoint @ " TARGET_ADDR_FMT,
		target_name(target),
		breakpoint->address);

	return ERROR_OK;
}

int xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint32_t slot;

	if (breakpoint->type == BKPT_SOFT) {
		for (slot = 0; slot < XT_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if (xtensa->sw_brps[slot].oocd_bp != NULL &&
				xtensa->sw_brps[slot].oocd_bp == breakpoint)
				break;
		}
		if (slot == XT_SW_BREAKPOINTS_MAX_NUM) {
			LOG_WARNING("%s: Max SW breakpoints slot reached, slot=%u!",
				target_name(target), slot);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		int ret = xtensa_sw_breakpoint_remove(target, &xtensa->sw_brps[slot]);
		if (ret != ERROR_OK) {
			LOG_ERROR("%s: Failed to remove SW breakpoint (%d)!",
				target_name(target),
				ret);
			return ret;
		}
		LOG_DEBUG("%s: cleared SW breakpoint %u @ " TARGET_ADDR_FMT,
			target_name(target),
			slot,
			breakpoint->address);
		return ERROR_OK;
	}

	for (slot = 0; slot < xtensa->core_config->debug.ibreaks_num; slot++) {
		if (xtensa->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.ibreaks_num) {
		LOG_WARNING("%s: HW breakpoint not found!", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	xtensa->hw_brps[slot] = NULL;
	LOG_DEBUG("%s: cleared HW breakpoint %u @ " TARGET_ADDR_FMT,
		target_name(target),
		slot,
		breakpoint->address);
	return ERROR_OK;
}

int xtensa_watchpoint_add(struct target *target, struct watchpoint *watchpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint32_t slot;
	xtensa_reg_val_t dbreakcval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->mask != ~(uint32_t)0) {
		LOG_ERROR("%s: watchpoint value masks not supported", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for (slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++) {
		if (xtensa->hw_wps[slot] == NULL || xtensa->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.dbreaks_num) {
		LOG_WARNING("%s: No free slots to add HW watchpoint!", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/*Figure out value for dbreakc5..0 */
	/*It's basically 0x3F with an incremental bit removed from the LSB for each extra length
	 * power of 2. */
	/*Yes, I could have calculated this, but for 7 options... */
	dbreakcval = 0xaa;	/*invalid, so we can check if this is changed later */
	if (watchpoint->length == 1 && (watchpoint->address & 0x00) == 0)
		dbreakcval = 0x3F;
	if (watchpoint->length == 2 && (watchpoint->address & 0x01) == 0)
		dbreakcval = 0x3E;
	if (watchpoint->length == 4 && (watchpoint->address & 0x03) == 0)
		dbreakcval = 0x3C;
	if (watchpoint->length == 8 && (watchpoint->address & 0x07) == 0)
		dbreakcval = 0x38;
	if (watchpoint->length == 16 && (watchpoint->address & 0x0F) == 0)
		dbreakcval = 0x30;
	if (watchpoint->length == 32 && (watchpoint->address & 0x1F) == 0)
		dbreakcval = 0x20;
	if (watchpoint->length == 64 && (watchpoint->address & 0x3F) == 0)
		dbreakcval = 0x00;
	if (dbreakcval == 0xaa) {
		LOG_WARNING(
			"%s: Watchpoint with length %d on address "TARGET_ADDR_FMT
			" not supported by hardware.",
			target_name(target),
			watchpoint->length,
			watchpoint->address);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (watchpoint->rw == WPT_READ)
		dbreakcval |= (1<<30);
	if (watchpoint->rw == WPT_WRITE)
		dbreakcval |= (1<<31);
	if (watchpoint->rw == WPT_ACCESS)
		dbreakcval |= (1<<30) + (1<<31);

	/* Write DBREAKA[slot] and DBCREAKC[slot]*/
	xtensa_reg_set(target, XT_REG_IDX_DBREAKA0 + slot, watchpoint->address);
	xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, dbreakcval);
	xtensa->hw_wps[slot] = watchpoint;
	LOG_DEBUG("%s: placed HW watchpoint @ " TARGET_ADDR_FMT,
		target_name(target),
		watchpoint->address);
	return ERROR_OK;
}

int xtensa_watchpoint_remove(struct target *target, struct watchpoint *watchpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint32_t slot;

	for (slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++) {
		if (xtensa->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.dbreaks_num) {
		LOG_WARNING("%s: HW watchpoint not found!", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, 0);
	xtensa->hw_wps[slot] = NULL;
	LOG_DEBUG("%s: cleared HW watchpoint @ " TARGET_ADDR_FMT,
		target_name(target),
		watchpoint->address);
	return ERROR_OK;
}

int xtensa_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct xtensa_algorithm *algorithm_info = arch_info;
	int retval = ERROR_OK;
	bool usr_ps = false;

	/* NOTE: xtensa_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Target not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache */
	for (unsigned i = 0; i < xtensa->core_cache->num_regs; i++)
		algorithm_info->context[i] = xtensa_reg_get(target, i);
	/* save debug reason, it will be changed */
	algorithm_info->ctx_debug_reason = target->debug_reason;
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
		struct reg *reg =
			register_get_by_name(xtensa->core_cache, reg_params[i].reg_name, 0);
		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
				reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (memcmp(reg_params[i].reg_name, "ps", 3))
			usr_ps = true;
		xtensa_reg_set_value(reg, buf_get_u32(reg_params[i].value, 0, 32));
		reg->valid = 1;
	}
	/* ignore custom core mode if custom PS value is specified */
	if (!usr_ps) {
		xtensa_reg_val_t ps = xtensa_reg_get(target, XT_REG_IDX_PS);
		enum xtensa_mode core_mode = XT_PS_RING_GET(ps);
		if (algorithm_info->core_mode != XT_MODE_ANY &&
			algorithm_info->core_mode != core_mode) {
			LOG_DEBUG("setting core_mode: 0x%x", algorithm_info->core_mode);
			xtensa_reg_val_t new_ps = (ps & ~XT_PS_RING_MSK) | XT_PS_RING(
				algorithm_info->core_mode);
			/* save previous core mode */
			algorithm_info->core_mode = core_mode;
			xtensa_reg_set(target, XT_REG_IDX_PS, new_ps);
			xtensa->core_cache->reg_list[XT_REG_IDX_PS].valid = 1;
		}
	}

	return xtensa_resume(target, 0, entry_point, 1, 1);
}

/** Waits for an algorithm in the target. */
int xtensa_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct xtensa_algorithm *algorithm_info = arch_info;
	int retval = ERROR_OK;
	xtensa_reg_val_t pc;

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
			xtensa_reg_get(target, XT_REG_IDX_PC),
			xtensa_reg_get(target, XT_REG_IDX_PS));
		return ERROR_TARGET_TIMEOUT;
	}
	pc = xtensa_reg_get(target, XT_REG_IDX_PC);
	if (exit_point && (pc != exit_point)) {
		LOG_ERROR("failed algorithm halted at 0x%" PRIx32 ", expected " TARGET_ADDR_FMT,
			pc,
			exit_point);
		return ERROR_TARGET_TIMEOUT;
	}
	/* Read memory values to mem_params */
	LOG_DEBUG("Read mem params");
	for (int i = 0; i < num_mem_params; i++) {
		LOG_DEBUG("Check mem param @ " TARGET_ADDR_FMT, mem_params[i].address);
		if (mem_params[i].direction != PARAM_OUT) {
			LOG_DEBUG("Read mem param @ " TARGET_ADDR_FMT, mem_params[i].address);
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
			struct reg *reg = register_get_by_name(xtensa->core_cache,
				reg_params[i].reg_name,
				0);
			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			if (reg->size != reg_params[i].size) {
				LOG_ERROR(
					"BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
			buf_set_u32(reg_params[i].value, 0, 32, xtensa_reg_get_value(reg));
		}
	}

	for (int i = xtensa->core_cache->num_regs - 1; i >= 0; i--) {
		xtensa_reg_val_t regvalue = xtensa_reg_get(target, i);
		if (i == XT_REG_IDX_DEBUGCAUSE) {
			/*FIXME: restoring DEBUGCAUSE causes exception when executing corresponding
			 * instruction in DIR */
			LOG_DEBUG("Skip restoring register %s: 0x%x -> 0x%8.8" PRIx32,
				xtensa->core_cache->reg_list[i].name,
				regvalue,
				algorithm_info->context[i]);
			xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);
			xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = 0;
			xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].valid = 0;
			continue;
		}
		if (regvalue != algorithm_info->context[i]) {
			LOG_DEBUG("restoring register %s: 0x%x -> 0x%8.8" PRIx32,
				xtensa->core_cache->reg_list[i].name,
				regvalue,
				algorithm_info->context[i]);
			xtensa_reg_set(target, i, algorithm_info->context[i]);
			xtensa->core_cache->reg_list[i].valid = 1;
		}
	}
	target->debug_reason = algorithm_info->ctx_debug_reason;

	retval = xtensa_write_dirty_registers(target);
	if (retval != ERROR_OK)
		LOG_ERROR("Failed to write dirty regs (%d)!", retval);

	return retval;
}

int xtensa_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	int timeout_ms, void *arch_info)
{
	int retval;

	retval = xtensa_start_algorithm(target,
		num_mem_params, mem_params,
		num_reg_params, reg_params,
		entry_point, exit_point,
		arch_info);

	if (retval == ERROR_OK)
		retval = xtensa_wait_algorithm(target,
			num_mem_params, mem_params,
			num_reg_params, reg_params,
			exit_point, timeout_ms,
			arch_info);

	return retval;
}

void xtensa_build_reg_cache(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);

	struct reg_cache *reg_cache = calloc(1, sizeof(struct reg_cache));
	if (reg_cache == NULL)
		return;
	reg_cache->name = "Xtensa registers";
	reg_cache->next = NULL;
	/* Init reglist */
	struct reg *reg_list = calloc(XT_NUM_REGS, sizeof(struct reg));
	if (reg_cache == NULL) {
		free(reg_cache);
		return;
	}
	xtensa->regs_num = 0;
	for (int i = 0; i < XT_NUM_REGS; i++) {
		reg_list[i].exist = false;
		if (xtensa_regs[i].type == XT_REG_USER) {
			if (xtensa_user_reg_exists(xtensa, i))
				reg_list[i].exist = true;
		} else if (xtensa_regs[i].type == XT_REG_FR) {
			if (xtensa_fp_reg_exists(xtensa, i))
				reg_list[i].exist = true;
		} else if (xtensa_regs[i].type == XT_REG_SPECIAL) {
			if (xtensa_special_reg_exists(xtensa, i))
				reg_list[i].exist = true;
		} else {
			if (xtensa_regular_reg_exists(xtensa, i))
				reg_list[i].exist = true;
		}
		reg_list[i].name = xtensa_regs[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, sizeof(uint32_t));
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &xtensa_reg_type;
		reg_list[i].arch_info = xtensa;
		if (reg_list[i].exist)
			xtensa->regs_num++;
	}
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = XT_NUM_REGS;

	xtensa->core_cache = reg_cache;
	(*cache_p) = reg_cache;
}

int xtensa_init_arch_info(struct target *target, struct xtensa *xtensa,
	const struct xtensa_config *xtensa_config,
	const struct xtensa_debug_module_config *dm_cfg,
	const struct xtensa_chip_ops *chip_ops)
{
	target->arch_info = xtensa;
	xtensa->target = target;
	xtensa->core_config = xtensa_config;
	xtensa->chip_ops = chip_ops;
	xtensa->stepping_isr_mode = XT_STEPPING_ISR_ON;

	if (!xtensa->core_config->exc.enabled || !xtensa->core_config->irq.enabled ||
		!xtensa->core_config->high_irq.enabled || !xtensa->core_config->debug.enabled) {
		LOG_ERROR("Xtensa configuration does not support debugging!");
		return ERROR_FAIL;
	}

	int ret = xtensa_dm_init(&xtensa->dbg_mod, dm_cfg);
	if (ret != ERROR_OK)
		return ret;
	xtensa->hw_brps =
		calloc(xtensa->core_config->debug.ibreaks_num, sizeof(struct breakpoint *));
	if (xtensa->hw_brps == NULL) {
		LOG_ERROR("Failed to alloc memory for HW breakpoints!");
		return ERROR_FAIL;
	}
	xtensa->hw_wps =
		calloc(xtensa->core_config->debug.dbreaks_num, sizeof(struct watchpoint *));
	if (xtensa->hw_wps == NULL) {
		free(xtensa->hw_brps);
		LOG_ERROR("Failed to alloc memory for HW watchpoints!");
		return ERROR_FAIL;
	}
	xtensa->sw_brps = calloc(XT_SW_BREAKPOINTS_MAX_NUM, sizeof(struct xtensa_sw_breakpoint));
	if (xtensa->sw_brps == NULL) {
		free(xtensa->hw_brps);
		free(xtensa->hw_wps);
		LOG_ERROR("Failed to alloc memory for SW breakpoints!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

COMMAND_HELPER(xtensa_cmd_permissive_mode_do, struct xtensa *xtensa)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	bool is_one = strcmp(CMD_ARGV[0], "1") == 0;
	if (!is_one && strcmp(CMD_ARGV[0], "0") != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;	/* 0 or 1 only */
	xtensa->permissive_mode = is_one;
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_permissive_mode)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_permissive_mode_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* perfmon_enable <counter_id> <select> [mask] [kernelcnt] [tracelevel] */
COMMAND_HELPER(xtensa_cmd_perfmon_enable_do, struct xtensa *xtensa)
{
	struct xtensa_perfmon_config config = {
		.mask = 0xffff,
		.kernelcnt = 0,
		.tracelevel = -1/* use DEBUGLEVEL by default */
	};

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	unsigned counter_id = strtoul(CMD_ARGV[0], NULL, 0);
	if (counter_id >= XTENSA_MAX_PERF_COUNTERS) {
		command_print(CMD, "counter_id should be < %d", XTENSA_MAX_PERF_COUNTERS);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	config.select = strtoul(CMD_ARGV[1], NULL, 0);
	if (config.select > XTENSA_MAX_PERF_SELECT) {
		command_print(CMD, "select should be < %d", XTENSA_MAX_PERF_SELECT);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC >= 3) {
		config.mask = strtoul(CMD_ARGV[2], NULL, 0);
		if (config.mask > XTENSA_MAX_PERF_MASK) {
			command_print(CMD, "mask should be < %d", XTENSA_MAX_PERF_MASK);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (CMD_ARGC >= 4) {
		config.kernelcnt = strtoul(CMD_ARGV[3], NULL, 0);
		if (config.kernelcnt > 1) {
			command_print(CMD, "kernelcnt should be 0 or 1");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (CMD_ARGC >= 5) {
		config.tracelevel = strtoul(CMD_ARGV[4], NULL, 0);
		if (config.tracelevel > 7) {
			command_print(CMD, "tracelevel should be <=7");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (config.tracelevel == -1)
		config.tracelevel = xtensa->core_config->debug.irq_level;
	return xtensa_dm_perfmon_enable(&xtensa->dbg_mod, counter_id, &config);
}

COMMAND_HANDLER(xtensa_cmd_perfmon_enable)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_enable_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* perfmon_dump [counter_id] */
COMMAND_HELPER(xtensa_cmd_perfmon_dump_do, struct xtensa *xtensa)
{
	int res;

	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	int counter_id = -1;
	if (CMD_ARGC == 1) {
		counter_id = strtol(CMD_ARGV[0], NULL, 0);
		if (counter_id > XTENSA_MAX_PERF_COUNTERS) {
			LOG_ERROR("counter_id should be < %d", XTENSA_MAX_PERF_COUNTERS);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	int counter_start = (counter_id < 0) ? 0 : counter_id;
	int counter_end = (counter_id < 0) ? XTENSA_MAX_PERF_COUNTERS : counter_id + 1;
	for (int counter = counter_start; counter < counter_end; ++counter) {
		char result_buf[128] = { 0 };
		size_t result_pos = 0;
		result_pos += snprintf(result_buf, sizeof(result_buf), "Counter %d: ", counter);
		struct xtensa_perfmon_result result;
		res = xtensa_dm_perfmon_dump(&xtensa->dbg_mod, counter, &result);
		if (res != ERROR_OK)
			return res;
		result_pos += snprintf(result_buf + result_pos, sizeof(result_buf) - result_pos,
			"%-12llu%s",
			(unsigned long long) result.value,
			(result.overflow) ? " (overflow)" : "");
		LOG_INFO("%s", result_buf);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_perfmon_dump)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_dump_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_mask_interrupts_do, struct xtensa *xtensa)
{
	int state = -1;

	if (CMD_ARGC < 1) {
		const char *st;
		state = xtensa->stepping_isr_mode;
		if (state == XT_STEPPING_ISR_ON)
			st = "OFF";
		else if (state == XT_STEPPING_ISR_OFF)
			st = "ON";
		else
			st = "UNKNOWN";
		command_print(CMD, "Current ISR step mode: %s", st);
		return ERROR_OK;
	}
	/* Masking is ON -> interrupts during stepping are OFF, and vice versa */
	if (!strcasecmp(CMD_ARGV[0], "off"))
		state = XT_STEPPING_ISR_ON;
	else if (!strcasecmp(CMD_ARGV[0], "on"))
		state = XT_STEPPING_ISR_OFF;

	if (state == -1) {
		command_print(CMD, "Argument unknown. Please pick one of ON, OFF");
		return ERROR_FAIL;
	}
	xtensa->stepping_isr_mode = state;
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_mask_interrupts)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_tracestart_do, struct xtensa *xtensa)
{
	int res;
	unsigned int i;
	struct xtensa_trace_start_config cfg = {
		.stoppc = 0,
		.stopmask = -1,
		.after = 0,
		.after_is_words = false
	};

	/*Parse arguments */
	for (i = 0; i < CMD_ARGC; i++) {
		if ((!strcasecmp(CMD_ARGV[i], "pc")) && CMD_ARGC > i) {
			char *e;
			i++;
			cfg.stoppc = strtol(CMD_ARGV[i], &e, 0);
			cfg.stopmask = 0;
			if (*e == '/')
				cfg.stopmask = strtol(e, NULL, 0);
		} else if ((!strcasecmp(CMD_ARGV[i], "after")) && CMD_ARGC > i) {
			i++;
			cfg.after = strtol(CMD_ARGV[i], NULL, 0);
		} else if (!strcasecmp(CMD_ARGV[i], "ins"))
			cfg.after_is_words = 0;
		else if (!strcasecmp(CMD_ARGV[i], "words"))
			cfg.after_is_words = 1;
		else {
			command_print(CMD, "Did not understand %s", CMD_ARGV[i]);
			return ERROR_FAIL;
		}
	}
	/* TODO: check this, it was copied from original ESP32/ESP108 implementation */
	cfg.stopmask = 1;

	res = xtensa_dm_trace_stop(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;

	res = xtensa_dm_trace_start(&xtensa->dbg_mod, &cfg);
	if (res != ERROR_OK)
		return res;

	xtensa->trace_active = true;
	command_print(CMD, "Trace started.");
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_tracestart)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestart_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_tracestop_do, struct xtensa *xtensa)
{
	struct xtensa_trace_status trace_status;

	int res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
	if (res != ERROR_OK)
		return res;

	if (!(trace_status.stat & TRAXSTAT_TRACT)) {
		command_print(CMD, "No trace is currently active.");
		return ERROR_FAIL;
	}

	res = xtensa_dm_trace_stop(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;

	xtensa->trace_active = false;
	command_print(CMD, "Trace stop triggered.");
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_tracestop)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestop_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_tracedump_do, struct xtensa *xtensa, const char *fname)
{
	struct xtensa_trace_config trace_config;
	struct xtensa_trace_status trace_status;
	uint32_t memsz, wmem;

	int res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
	if (res != ERROR_OK)
		return res;

	if (trace_status.stat & TRAXSTAT_TRACT) {
		command_print(CMD, "Tracing is still active. Please stop it first.");
		return ERROR_FAIL;
	}

	res = xtensa_dm_trace_config_read(&xtensa->dbg_mod, &trace_config);
	if (res != ERROR_OK)
		return res;

	if (!(trace_config.ctrl & TRAXCTRL_TREN)) {
		command_print(CMD, "No active trace found; nothing to dump.");
		return ERROR_FAIL;
	}

	memsz = trace_config.memaddr_end - trace_config.memaddr_start + 1;
	LOG_INFO("Total trace memory: %d words", memsz);
	if ((trace_config.addr &
			((TRAXADDR_TWRAP_MASK << TRAXADDR_TWRAP_SHIFT) | TRAXADDR_TWSAT)) == 0) {
		/*Memory hasn't overwritten itself yet. */
		wmem = trace_config.addr & TRAXADDR_TADDR_MASK;
		LOG_INFO("...but trace is only %d words", wmem);
		if (wmem < memsz)
			memsz = wmem;
	} else {
		if (trace_config.addr & TRAXADDR_TWSAT)
			LOG_INFO("Real trace is many times longer than that (overflow)");
		else {
			uint32_t trc_sz = (trace_config.addr >> TRAXADDR_TWRAP_SHIFT) &
				TRAXADDR_TWRAP_MASK;
			trc_sz = (trc_sz * memsz) + (trace_config.addr & TRAXADDR_TADDR_MASK);
			LOG_INFO("Real trace is %d words, but the start has been truncated.",
				trc_sz);
		}
	}

	uint8_t *tracemem = malloc(memsz*4);
	if (tracemem == NULL) {
		command_print(CMD, "Failed to alloc memory for trace data!");
		return ERROR_FAIL;
	}
	res = xtensa_dm_trace_data_read(&xtensa->dbg_mod, tracemem, memsz*4);
	if (res != ERROR_OK)
		return res;

	int f = open(fname, O_WRONLY|O_CREAT|O_TRUNC, 0666);
	if (f <= 0) {
		free(tracemem);
		command_print(CMD, "Unable to open file %s", fname);
		return ERROR_FAIL;
	}
	if (write(f, tracemem, memsz*4) != (int)memsz*4)
		command_print(CMD, "Unable to write to file %s", fname);
	else
		command_print(CMD, "Written %d bytes of trace data to %s", memsz*4, fname);
	close(f);

	bool is_all_zeroes = true;
	for (uint32_t i = 0; i < memsz*4; i++) {
		if (tracemem[i] != 0) {
			is_all_zeroes = false;
			break;
		}
	}
	if (is_all_zeroes)
		command_print(
			CMD,
			"WARNING: File written is all zeroes. Are you sure you enabled trace memory?");

	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_tracedump)
{
	if (CMD_ARGC < 1) {
		command_print(CMD, "Need filename to dump to as output!");
		return ERROR_FAIL;
	}

	return CALL_COMMAND_HANDLER(xtensa_cmd_tracedump_do,
		target_to_xtensa(get_current_target(CMD_CTX)), CMD_ARGV[0]);
}

const struct command_registration xtensa_command_handlers[] = {
	{
		.name = "set_permissive",
		.handler = xtensa_cmd_permissive_mode,
		.mode = COMMAND_ANY,
		.help = "When set to 1, enable ESP108 permissive mode (less client-side checks)",
		.usage = "[0|1]",
	},
	{
		.name = "maskisr",
		.handler = xtensa_cmd_mask_interrupts,
		.mode = COMMAND_ANY,
		.help = "mask Xtensa interrupts at step",
		.usage = "['on'|'off']",
	},
	{
		.name = "perfmon_enable",
		.handler = xtensa_cmd_perfmon_enable,
		.mode = COMMAND_EXEC,
		.help = "Enable and start performance counter",
		.usage = "<counter_id> <select> [mask] [kernelcnt] [tracelevel]",
	},
	{
		.name = "perfmon_dump",
		.handler = xtensa_cmd_perfmon_dump,
		.mode = COMMAND_EXEC,
		.help =
			"Dump performance counter value. If no argument specified, dumps all counters.",
		.usage = "[counter_id]",
	},
	{
		.name = "tracestart",
		.handler = xtensa_cmd_tracestart,
		.mode = COMMAND_EXEC,
		.help =
			"Tracing: Set up and start a trace. Optionally set stop trigger address and amount of data captured after.",
		.usage = "[pc <pcval>/[maskbitcount]] [after <n> [ins|words]]",
	},
	{
		.name = "tracestop",
		.handler = xtensa_cmd_tracestop,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Stop current trace as started by the tracestart command",
		.usage = "",
	},
	{
		.name = "tracedump",
		.handler = xtensa_cmd_tracedump,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Dump trace memory to a files. One file per core.",
		.usage = "<outfile>",
	},
	COMMAND_REGISTRATION_DONE
};
