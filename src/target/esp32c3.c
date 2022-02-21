/***************************************************************************
 *   ESP32-C3 target for OpenOCD                                           *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "esp32c3.h"
#include <helper/command.h>
#include <helper/bits.h>
#include "target_type.h"
#include "register.h"
#include "riscv/debug_defines.h"
#include "esp32_apptrace.h"
#include "rtos/rtos.h"

/* ESP32-C3 WDT */
#define ESP32C3_WDT_WKEY_VALUE       0x50d83aa1
#define ESP32C3_TIMG0_BASE           0x6001F000
#define ESP32C3_TIMG1_BASE           0x60020000
#define ESP32C3_TIMGWDT_CFG0_OFF     0x48
#define ESP32C3_TIMGWDT_PROTECT_OFF  0x64
#define ESP32C3_TIMG0WDT_CFG0        (ESP32C3_TIMG0_BASE + ESP32C3_TIMGWDT_CFG0_OFF)
#define ESP32C3_TIMG1WDT_CFG0        (ESP32C3_TIMG1_BASE + ESP32C3_TIMGWDT_CFG0_OFF)
#define ESP32C3_TIMG0WDT_PROTECT     (ESP32C3_TIMG0_BASE + ESP32C3_TIMGWDT_PROTECT_OFF)
#define ESP32C3_TIMG1WDT_PROTECT     (ESP32C3_TIMG1_BASE + ESP32C3_TIMGWDT_PROTECT_OFF)
#define ESP32C3_RTCCNTL_BASE         0x60008000
#define ESP32C3_RTCWDT_CFG_OFF       0x90
#define ESP32C3_RTCWDT_PROTECT_OFF   0xa8
#define ESP32C3_RTCWDT_CFG           (ESP32C3_RTCCNTL_BASE + ESP32C3_RTCWDT_CFG_OFF)
#define ESP32C3_RTCWDT_PROTECT       (ESP32C3_RTCCNTL_BASE + ESP32C3_RTCWDT_PROTECT_OFF)

#define ESP32C3_GPIO_STRAP_REG      0x60004038UL
#define IS_1XXX(v)                  (((v)&0x08) == 0x08)
#define ESP32C3_IS_FLASH_BOOT(_r_)  IS_1XXX(_r_)
#define ESP32C3_FLASH_BOOT_MODE     0x08

extern struct target_type riscv_target;
extern const struct command_registration riscv_command_handlers[];

static int esp32c3_on_reset(struct target *target);


static int esp32c3_wdt_disable(struct target *target)
{
	/* TIMG1 WDT */
	int res = target_write_u32(target, ESP32C3_TIMG0WDT_PROTECT, ESP32C3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG0WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C3_TIMG0WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG0WDT_CFG0 (%d)!", res);
		return res;
	}
	/* TIMG2 WDT */
	res = target_write_u32(target, ESP32C3_TIMG1WDT_PROTECT, ESP32C3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG1WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C3_TIMG1WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG1WDT_CFG0 (%d)!", res);
		return res;
	}
	/* RTC WDT */
	res = target_write_u32(target, ESP32C3_RTCWDT_PROTECT, ESP32C3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_RTCWDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C3_RTCWDT_CFG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_RTCWDT_CFG (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static const struct esp_semihost_ops esp32c3_semihost_ops = {
	.prepare = esp32c3_wdt_disable
};

static const struct esp_flash_breakpoint_ops esp32c3_flash_brp_ops = {
	.breakpoint_add = esp_flash_breakpoint_add,
	.breakpoint_remove = esp_flash_breakpoint_remove
};

static int esp32c3_handle_target_event(struct target *target, enum target_event event, void *priv)
{
	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);

	int ret = esp_riscv_handle_target_event(target, event, priv);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int esp32c3_target_create(struct target *target, Jim_Interp *interp)
{
	struct esp32c3_common *esp32c3 = calloc(1, sizeof(struct esp32c3_common));
	if (!esp32c3)
		return ERROR_FAIL;

	target->arch_info = esp32c3;

	riscv_info_init(target, &esp32c3->esp_riscv.riscv);

	return ERROR_OK;
}

static int esp32c3_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	struct esp32c3_common *esp32c3 = esp32c3_common(target);

	ret = esp_riscv_init_arch_info(cmd_ctx,
		target,
		&esp32c3->esp_riscv,
		esp32c3_on_reset,
		&esp32c3_flash_brp_ops,
		&esp32c3_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	ret = target_register_event_callback(esp32c3_handle_target_event, target);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static void esp32c3_deinit_target(struct target *target)
{
	riscv_target.deinit_target(target);
}

static const char *s_nonexistent_regs[] = {
	"mie", "mip", "tdata3", "uie", "utvt", "utvec", "vcsr", "uscratch", "utval",
	"uip", "unxti", "uintstatus", "uscratchcsw", "uscratchcswl", "sedeleg",
	"sideleg", "stvt", "snxti", "sintstatus", "sscratchcsw", "sscratchcswl",
	"vsstatus", "vsie", "vstvec", "vsscratch", "vsepc", "vscause", "vstval",
	"vsip", "vsatp", "mtvt", "mstatush", "mcountinhibit", "mnxti", "mintstatus",
	"mscratchcsw", "mscratchcswl", "mtinst", "mtval2", "hstatus", "hedeleg",
	"hideleg", "hie", "htimedelta", "hcounteren", "hgeie", "htimedeltah",
	"htval", "hip", "hvip", "htinst", "hgatp", "hgeip", "mvendorid", "marchid",
	"mimpid", "mhartid", "seed", "mcounteren", "mhpmevent3", "mhpmevent4", "mhpmevent5",
	"mhpmevent6", "mhpmevent7", "mhpmevent8", "mhpmevent9", "mhpmevent10", "mhpmevent11",
	"mhpmevent12", "mhpmevent13", "mhpmevent14", "mhpmevent15", "mhpmevent16", "mhpmevent17",
	"mhpmevent18", "mhpmevent19", "mhpmevent20", "mhpmevent21", "mhpmevent22", "mhpmevent23",
	"mhpmevent24", "mhpmevent25", "mhpmevent26", "mhpmevent27", "mhpmevent28", "mhpmevent29",
	"mhpmevent30", "mhpmevent31",
	"scontext", "hcontext", "tinfo", "mcontext", "mscontext", "mcycle", "minstret",
	"mhpmcounter3", "mhpmcounter4", "mhpmcounter5", "mhpmcounter6", "mhpmcounter7",
	"mhpmcounter8", "mhpmcounter9", "mhpmcounter10", "mhpmcounter11", "mhpmcounter12",
	"mhpmcounter13", "mhpmcounter14", "mhpmcounter15", "mhpmcounter16", "mhpmcounter17",
	"mhpmcounter18", "mhpmcounter19", "mhpmcounter20", "mhpmcounter21", "mhpmcounter22",
	"mhpmcounter23", "mhpmcounter24", "mhpmcounter25", "mhpmcounter26", "mhpmcounter27",
	"mhpmcounter28", "mhpmcounter29", "mhpmcounter30", "mhpmcounter31", "mcycleh",
	"minstreth", "mhpmcounter3h", "mhpmcounter4h", "mhpmcounter5h", "mhpmcounter6h",
	"mhpmcounter7h", "mhpmcounter8h", "mhpmcounter9h", "mhpmcounter10h", "mhpmcounter11h",
	"mhpmcounter12h", "mhpmcounter13h", "mhpmcounter14h", "mhpmcounter15h", "mhpmcounter16h",
	"mhpmcounter17h", "mhpmcounter18h", "mhpmcounter19h", "mhpmcounter20h", "mhpmcounter21h",
	"mhpmcounter22h", "mhpmcounter23h", "mhpmcounter24h", "mhpmcounter25h", "mhpmcounter26h",
	"mhpmcounter27h", "mhpmcounter28h", "mhpmcounter29h", "mhpmcounter30h", "mhpmcounter31h",
	"cycle", "time", "instret", "hpmcounter3", "hpmcounter4", "hpmcounter5", "hpmcounter6",
	"hpmcounter7", "hpmcounter8", "hpmcounter9", "hpmcounter10", "hpmcounter11", "hpmcounter12",
	"hpmcounter13", "hpmcounter14", "hpmcounter15", "hpmcounter17", "hpmcounter18",
	"hpmcounter19",
	"hpmcounter20", "hpmcounter21", "hpmcounter22", "hpmcounter23", "hpmcounter24",
	"hpmcounter25",
	"hpmcounter26", "hpmcounter27", "hpmcounter28", "hpmcounter29", "hpmcounter30",
	"hpmcounter31",
	"cycleh", "timeh", "instreth", "hpmcounter3h", "hpmcounter4h", "hpmcounter5h",
	"hpmcounter6h",
	"hpmcounter7h", "hpmcounter8h", "hpmcounter9h", "hpmcounter10h", "hpmcounter11h",
	"hpmcounter12h",
	"hpmcounter13h", "hpmcounter14h", "hpmcounter15h", "hpmcounter17h", "hpmcounter18h",
	"hpmcounter19h",
	"hpmcounter20h", "hpmcounter21h", "hpmcounter22h", "hpmcounter23h", "hpmcounter24h",
	"hpmcounter25h",
	"hpmcounter26h", "hpmcounter27h", "hpmcounter28h", "hpmcounter29h", "hpmcounter30h",
	"hpmcounter31h",
	"hpmcounter16h", "mhpmevent4"
};

static int esp32c3_examine(struct target *target)
{
	int ret = riscv_target.examine(target);
	if (ret != ERROR_OK)
		return ret;
	/* RISCV code initializes registers upon target examination.
	   disable some registers because their reading or writing causes exception. Not supported in ESP32-C3??? */
	for (size_t i = 0; i < sizeof(s_nonexistent_regs)/sizeof(s_nonexistent_regs[0]); i++) {
		struct reg *r = register_get_by_name(target->reg_cache, s_nonexistent_regs[i], 1);
		if (r)
			r->exist = false;
	}
	return ERROR_OK;
}

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

static bool esp32c3_core_is_halted(struct target *target)
{
	uint32_t dmstatus;
	RISCV_INFO(r);

	if (r->dmi_read(target, &dmstatus, DM_DMSTATUS) != ERROR_OK)
		return false;
	return get_field(dmstatus, DM_DMSTATUS_ALLHALTED);
}

static int esp32c3_core_halt(struct target *target)
{
	RISCV_INFO(r);

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_HALTREQ;
	r->dmi_write(target, DM_DMCONTROL, dmcontrol);
	for (size_t i = 0; i < 256; ++i)
		if (esp32c3_core_is_halted(target))
			break;

	if (!esp32c3_core_is_halted(target)) {
		uint32_t dmstatus;
		if (r->dmi_read(target, &dmstatus, DM_DMSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if (r->dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
			return ERROR_FAIL;

		LOG_ERROR("unable to halt core");
		LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
		LOG_ERROR("  dmstatus =0x%08x", dmstatus);
		return ERROR_FAIL;
	}

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HALTREQ, 0);
	r->dmi_write(target, DM_DMCONTROL, dmcontrol);
	return ERROR_OK;
}

static int esp32c3_core_resume(struct target *target)
{
	RISCV_INFO(r);

	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_RESUMEREQ;
	r->dmi_write(target, DM_DMCONTROL, dmcontrol);

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HASEL, 0);
	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_RESUMEREQ, 0);

	uint32_t dmstatus;
	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		int res = r->dmi_read(target, &dmstatus, DM_DMSTATUS);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read dmstatus!");
			return res;
		}
		if (get_field(dmstatus, DM_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		res = r->dmi_write(target, DM_DMCONTROL, dmcontrol);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to write dmcontrol!");
			return res;
		}
		return ERROR_OK;
	}

	r->dmi_write(target, DM_DMCONTROL, dmcontrol);

	LOG_ERROR("unable to resume core");
	if (r->dmi_read(target, &dmstatus, DM_DMSTATUS) != ERROR_OK)
		return ERROR_FAIL;
	LOG_ERROR("  dmstatus =0x%08x", dmstatus);

	return ERROR_FAIL;
}

static int esp32c3_on_reset(struct target *target)
{
	LOG_DEBUG("esp32c3_on_reset!");
	struct esp32c3_common *esp32c3 = esp32c3_common(target);
	esp32c3->was_reset = true;
	return ERROR_OK;
}

static int esp32c3_poll(struct target *target)
{
	struct esp32c3_common *esp32c3 = esp32c3_common(target);
	int res = ERROR_OK;

	RISCV_INFO(r);
	if (esp32c3->was_reset && r->dmi_read && r->dmi_write) {
		uint32_t dmstatus;
		res = r->dmi_read(target, &dmstatus, DM_DMSTATUS);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to read DMSTATUS (%d)!", res);
		else {
			uint32_t strap_reg;
			LOG_DEBUG("Core is out of reset: dmstatus 0x%x", dmstatus);
			esp32c3->was_reset = false;
			res = target_read_u32(target, ESP32C3_GPIO_STRAP_REG, &strap_reg);
			if (res != ERROR_OK) {
				LOG_WARNING("Failed to read ESP32C3_GPIO_STRAP_REG (%d)!", res);
				strap_reg = ESP32C3_FLASH_BOOT_MODE;
			}
			if (ESP32C3_IS_FLASH_BOOT(strap_reg) &&
				get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0) {
				LOG_DEBUG("Halt core");
				res = esp32c3_core_halt(target);
				if (res == ERROR_OK) {
					res = esp32c3_wdt_disable(target);
					if (res != ERROR_OK)
						LOG_ERROR("Failed to disable WDTs (%d)!", res);
				} else
					LOG_ERROR("Failed to halt core (%d)!", res);
			}
			/* Clear memory which is used by RTOS layer to get the task count */
			if (target->rtos && target->rtos->type->post_reset_cleanup) {
				res = (*target->rtos->type->post_reset_cleanup)(target);
				if (res != ERROR_OK)
					LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
			}
			if (ESP32C3_IS_FLASH_BOOT(strap_reg)) {
				if (get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0) {
					LOG_DEBUG("Resume core");
					res = esp32c3_core_resume(target);
					if (res != ERROR_OK)
						LOG_ERROR("Failed to resume core (%d)!", res);
					LOG_DEBUG("resumed core");
				}
			}
		}
	}
	return riscv_target.poll(target);
}

static int esp32c3_halt(struct target *target)
{
	return riscv_target.halt(target);
}

static int esp32c3_resume(struct target *target, int current, target_addr_t address,
	int handle_breakpoints, int debug_execution)
{
	return riscv_target.resume(target, current, address, handle_breakpoints, debug_execution);
}

static int esp32c3_step(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	return riscv_target.step(target, current, address, handle_breakpoints);
}

static int esp32c3_assert_reset(struct target *target)
{
	return riscv_target.assert_reset(target);
}

static int esp32c3_deassert_reset(struct target *target)
{
	return riscv_target.deassert_reset(target);
}

static int esp32c3_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	/* TODO: find out the widest system bus access size. For now we are assuming it is
	        equal to xlen */
	uint32_t sba_access_size = target_data_bits(target) / 8;

	if (size < sba_access_size) {
		LOG_DEBUG("Use %d-bit access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, sba_access_size * 8, size, count, address);
		target_addr_t al_addr = address & ~(sba_access_size - 1);
		uint32_t al_cnt = 4 * ((size * count) / sba_access_size + 1);
		uint8_t al_buf[al_cnt];
		int ret = riscv_target.read_memory(target,
			al_addr,
			sba_access_size,
			al_cnt / sba_access_size,
			al_buf);
		if (ret == ERROR_OK)
			memcpy(buffer, &al_buf[address & (sba_access_size - 1)], size * count);
		return ret;
	}

	return riscv_target.read_memory(target, address, size, count, buffer);
}

static int esp32c3_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/* TODO: find out the widest system bus access size. For now we are assuming it is
	        equal to xlen */
	uint32_t sba_access_size = target_data_bits(target) / 8;

	if (target->state == TARGET_RUNNING || target->state == TARGET_DEBUG_RUNNING) {
		/* Emulate using 32-bit SBA access if target is running.
		   Access via prog_buf or abstartct commands does not work in running state and
		   fails with abstractcs.cmderr == 4 (halt/resume) */
		if (size < sba_access_size) {
			LOG_DEBUG("Use %d-bit access: size: %d\tcount:%d\tstart address: 0x%08"
				TARGET_PRIxADDR, sba_access_size * 8, size, count, address);
			target_addr_t al_addr = address & ~(sba_access_size - 1);
			uint32_t al_cnt = 4 * ((size * count) / sba_access_size + 1);
			uint8_t al_buf[al_cnt];
			int ret = riscv_target.read_memory(target,
				al_addr,
				sba_access_size,
				al_cnt / sba_access_size,
				al_buf);
			if (ret == ERROR_OK) {
				memcpy(&al_buf[address & (sba_access_size - 1)],
					buffer,
					size * count);
				ret = riscv_target.write_memory(target,
					address,
					sba_access_size,
					al_cnt / sba_access_size,
					al_buf);
			}
			return ret;
		}
	}
	return riscv_target.write_memory(target, address, size, count, buffer);
}


static int esp32c3_checksum_memory(struct target *target,
	target_addr_t address, uint32_t count,
	uint32_t *checksum)
{
	return riscv_target.checksum_memory(target, address, count, checksum);
}

static int esp32c3_get_gdb_reg_list_noread(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	return riscv_target.get_gdb_reg_list_noread(target, reg_list, reg_list_size, reg_class);
}

static int esp32c3_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	return riscv_target.get_gdb_reg_list(target, reg_list, reg_list_size, reg_class);
}

static const char *esp32c3_get_gdb_arch(struct target *target)
{
	return riscv_target.get_gdb_arch(target);
}

static int esp32c3_arch_state(struct target *target)
{
	return riscv_target.arch_state(target);
}

static int esp32c3_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	return riscv_target.add_watchpoint(target, watchpoint);
}

static int esp32c3_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	return riscv_target.remove_watchpoint(target, watchpoint);
}

static int esp32c3_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	return riscv_target.hit_watchpoint(target, hit_watchpoint);
}

static unsigned esp32c3_address_bits(struct target *target)
{
	return riscv_target.address_bits(target);
}

static const struct command_registration esp32c3_command_handlers[] = {
	{
		.usage = "",
		.chain = riscv_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type esp32c3_target = {
	.name = "esp32c3",

	.target_create = esp32c3_target_create,
	.init_target = esp32c3_init_target,
	.deinit_target = esp32c3_deinit_target,
	.examine = esp32c3_examine,

	/* poll current target status */
	.poll = esp32c3_poll,

	.halt = esp32c3_halt,
	.resume = esp32c3_resume,
	.step = esp32c3_step,

	.assert_reset = esp32c3_assert_reset,
	.deassert_reset = esp32c3_deassert_reset,

	.read_memory = esp32c3_read_memory,
	.write_memory = esp32c3_write_memory,

	.checksum_memory = esp32c3_checksum_memory,

	.get_gdb_arch = esp32c3_get_gdb_arch,
	.get_gdb_reg_list = esp32c3_get_gdb_reg_list,
	.get_gdb_reg_list_noread = esp32c3_get_gdb_reg_list_noread,

	.add_breakpoint = esp_riscv_breakpoint_add,
	.remove_breakpoint = esp_riscv_breakpoint_remove,

	.add_watchpoint = esp32c3_add_watchpoint,
	.remove_watchpoint = esp32c3_remove_watchpoint,
	.hit_watchpoint = esp32c3_hit_watchpoint,

	.arch_state = esp32c3_arch_state,

	.run_algorithm = esp_riscv_run_algorithm,
	.start_algorithm = esp_riscv_start_algorithm,
	.wait_algorithm = esp_riscv_wait_algorithm,

	.commands = esp32c3_command_handlers,

	.address_bits = esp32c3_address_bits,
};
