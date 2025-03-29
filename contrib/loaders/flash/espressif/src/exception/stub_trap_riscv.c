// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <stddef.h>
#include <stdint.h>

#include <esp-stub-lib/log.h>

#include "esp_stub.h"

extern volatile union esp_stub_trap_record g_stub_trap_record;

extern void stub_trap_entry(void); /* defined in stub_trap_riscv_entry.S */

static inline uint32_t stub_trap_read_mcause(void)
{
	uint32_t v;
	__asm__ volatile ("csrr %0, mcause" : "=r"(v));
	return v;
}

static inline uint32_t stub_trap_read_mepc(void)
{
	uint32_t v;
	__asm__ volatile ("csrr %0, mepc" : "=r"(v));
	return v;
}

static inline uint32_t stub_trap_read_mtval(void)
{
	uint32_t v;
	__asm__ volatile ("csrr %0, mtval" : "=r"(v));
	return v;
}

static inline uint32_t stub_trap_read_mstatus(void)
{
	uint32_t v;
	__asm__ volatile ("csrr %0, mstatus" : "=r"(v));
	return v;
}

static inline uint32_t stub_trap_read_mhartid(void)
{
	uint32_t v;
	__asm__ volatile ("csrr %0, mhartid" : "=r"(v));
	return v;
}

void __attribute__((noreturn)) stub_trap_handler_c(uint32_t sp_at_trap, uint32_t ra_at_trap)
{
	volatile union esp_stub_trap_record *rec = &g_stub_trap_record;

	rec->magic = ESP_STUB_TRAP_RECORD_MAGIC;
	rec->version = ESP_STUB_TRAP_RECORD_VERSION;
	rec->flags = ESP_STUB_TRAP_RECORD_RISCV;
	rec->mcause = stub_trap_read_mcause();
	rec->mepc = stub_trap_read_mepc();
	rec->mtval = stub_trap_read_mtval();
	rec->mstatus = stub_trap_read_mstatus();
	rec->sp = sp_at_trap;
	rec->ra = ra_at_trap;
	rec->hartid = stub_trap_read_mhartid();

	__asm__ volatile ("li a0, %0\n\t"
		"fence iorw, iorw\n\t"
		"ebreak"
		:: "i"(ESP_STUB_ERR_EXCEPTION) : "a0", "memory");
	for (;;)
		__asm__ volatile ("wfi");
}
