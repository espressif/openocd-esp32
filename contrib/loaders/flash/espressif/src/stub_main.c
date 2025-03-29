// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

#include <esp-stub-lib/flash.h>
#include <esp-stub-lib/cache.h>
#include <esp-stub-lib/mmu.h>
#include <esp-stub-lib/log.h>
#include <esp-stub-lib/bit_utils.h>
#include <esp-stub-lib/err.h>
#include <esp-stub-lib/miniz.h>
#include <esp-stub-lib/security.h>
#include <esp-stub-lib/sha256.h>
#include <esp-stub-lib/clock.h>

#include "esp_stub.h"
#include "stub_apptrace.h"
#include "flash_mapping.h"
#include "stub_xtensa.h"
#include "stub_riscv.h"

extern uint32_t _bss_start;
extern uint32_t _bss_end;

struct stub_runtime_state {
	void *flash_state;
	void *cache_state;
};

/* Filled by stub_trap_handler */
volatile union esp_stub_trap_record g_stub_trap_record;

#define __maybe_unused __attribute__((unused))

/* --- Includes and globals --- */

const struct esp_stub_desc __attribute__((section(".stub_desc")))  s_esp_stub_desc = {
	.magic = ESP_STUB_FLASHER_MAGIC_NUM,
	.stub_version = ESP_STUB_FLASHER_VERSION,
#ifdef STUB_CMD_FLASH_IDF_BINARY
	.idf_key = ESP_STUB_FLASHER_IDF_KEY,
#else
	.idf_key = 0x00,
#endif
};

struct stub_cmd_handler {
	int cmd;
	const char *name;
	int (*handler)(va_list ap);
};

/* State for streaming flash write and deflate. */
static struct {
	/* offset of next flash write */
	uint32_t next_write;
	/* number of output bytes remaining to write */
	uint32_t remaining_uncompressed;
	/* number of compressed bytes remaining to read */
	uint32_t remaining_compressed;
	/* inflator state for deflate write */
	tinfl_decompressor *inflator;
	/* unzip buffer */
	uint8_t *out_buf;
	uint8_t *next_out;
} s_fs;

static uint32_t s_encrypt_binary;

/* --- Test & apptrace --- */

static __maybe_unused int handle_test1(va_list ap)
{
	(void)ap;

	uint32_t __maybe_unused addr = 0x4080393C;

	STUB_LOG("stub command test:%d\n", 1);
	STUB_LOG("stub command test:%d\n", -1);
	STUB_LOG("stub command test:0x%x\n", addr);
	STUB_LOG("stub command test:%s\n", "test");
	STUB_LOG("stub command test:%c\n", 'A');

	STUB_LOGE("stub command test\n");
	STUB_LOGW("stub command test\n");
	STUB_LOGI("stub command test\n");
	STUB_LOGD("stub command test\n");
	STUB_LOGV("stub command test\n");
	STUB_LOG_TRACE();
	STUB_LOG_TRACEF("foo:%u\n", 0x2A);

	stub_abort();
}

static int stub_apptrace_process_recvd_data(const __maybe_unused uint8_t *buf, uint32_t size)
{
	STUB_LOGD("apptrace process data: %x, size: %d\n", buf, size);

	for (uint32_t i = 0; i < MIN(size, 16); i++)
		STUB_LOG("%x ", buf[i]);

	STUB_LOG("\n");

	return ESP_STUB_OK;
}

static __maybe_unused int handle_apptrace_read_from_host(va_list ap)
{
	void *arg1 = va_arg(ap, void *);

	STUB_LOGD("apptrace read from host arg ptr: %x\n", (uint32_t)arg1);

	return stub_apptrace_recv_data(arg1, stub_apptrace_process_recvd_data);
}

static int stub_apptrace_process_write_data(uint32_t __maybe_unused addr, uint8_t *buf, uint32_t size)
{
	STUB_LOGV("apptrace write to host addr: %x, size: %d\n", addr, size);

	static uint8_t data;

	//prepare data to send to host
	for (uint32_t i = 0; i < size; i++)
		buf[i] = data++;

	return ESP_STUB_OK;
}

static __maybe_unused int handle_apptrace_write_to_host(va_list ap)
{
	uint32_t arg1 = va_arg(ap, uint32_t);   //address
	uint32_t arg2 = va_arg(ap, uint32_t);   //size

	STUB_LOGV("apptrace read from host arg ptr: %x\n", arg1);

	return stub_apptrace_send_data(arg1, arg2, stub_apptrace_process_write_data);
}

/* --- Flash: read --- */

static __maybe_unused int print_flash_info(void)
{
	stub_lib_flash_config_t flash_config;
	stub_lib_flash_get_config(&flash_config);

	STUB_LOGD("flash_id: 0x%x, fs: %d MB, bs: %d KB, ss: %d KB, ps: %d bytes, sm: 0x%x\n",
		flash_config.flash_id,
		BYTES_TO_MIB(flash_config.flash_size),
		BYTES_TO_KIB(flash_config.block_size),
		BYTES_TO_KIB(flash_config.sector_size),
		flash_config.page_size,
		flash_config.status_mask);

	return ESP_STUB_OK;
}

static int stub_read_data(uint32_t addr, uint8_t *buf, uint32_t size)
{
	STUB_LOG_TRACEF("addr: %x, size: %d\n", addr, size);

	return stub_lib_mmu_read_flash(addr, buf, size);
}

static __maybe_unused int handle_flash_read(va_list ap)
{
	uint32_t addr = va_arg(ap, uint32_t);
	uint32_t size = va_arg(ap, uint32_t);

	STUB_LOG_TRACEF("start addr: %x, size: %d\n", addr, size);

	return stub_apptrace_send_data(addr, size, stub_read_data);
}

/* --- Flash: write --- */

static __maybe_unused bool stub_encryption_is_enabled(void)
{
	static bool first;
	static bool encryption_enabled;

	if (!first) {
		first = true;
		encryption_enabled = stub_lib_security_flash_is_encrypted();
	}

	STUB_LOGV("Flash encryption enabled: %d\n", encryption_enabled);

	return encryption_enabled;
}

static int stub_spiflash_write(uint32_t spi_flash_addr, uint32_t *data, uint32_t len, bool encrypt)
{
	/*
	We can ask hardware to encrypt binary as long as flash encryption is enabled and encrypt option is set.
	During breakpoint set and clear, we will set 'encrypt' flag according to efuse flash encryption settings.
	So that hardware will handle the encryption if necessary.
	During flash programming, 'encrypt' flag will reflect the user specified 'encrypt_binary' programming option.
	If this is not set, we assume the binary is encrypted by the user.
	If this is set but flash encryption is not enabled, by returning an error,
	we will warn the user to do the right operation.
	*/
	if (encrypt && !stub_encryption_is_enabled())
		return ESP_STUB_ERR_FLASH_ENCRYPTION_NOT_ENABLED;

	int rc = stub_lib_flash_write_buff(spi_flash_addr, data, len, encrypt);

	STUB_LOGV("Write %sflash @ 0x%x sz %d\n", encrypt ? "encrypted-" : "", spi_flash_addr, len);

	return rc;
}

static __maybe_unused int stub_write_aligned_buffer(const void *data_buf, uint32_t length)
{
	while (length > 0 && s_fs.remaining_uncompressed > 0) {
		uint32_t out_bytes = length;
		uint32_t bytes_in_out_buf = (uint32_t)(s_fs.next_out - s_fs.out_buf);

		if (out_bytes + bytes_in_out_buf > ESP_STUB_UNZIP_BUFF_SIZE)
			out_bytes = ESP_STUB_UNZIP_BUFF_SIZE - bytes_in_out_buf;

		memcpy(s_fs.out_buf + bytes_in_out_buf, data_buf, out_bytes);

		bytes_in_out_buf += out_bytes;
		length -= out_bytes;
		data_buf += out_bytes;
		s_fs.next_out += out_bytes;

		if (s_fs.remaining_uncompressed - bytes_in_out_buf == 0 ||
			bytes_in_out_buf == ESP_STUB_UNZIP_BUFF_SIZE) {
			uint32_t wr_sz = bytes_in_out_buf;

			/* stub_spiflash_write() expects length to be aligned to 4 bytes.
			   If this is the last package we do not need to care about it here because OpenOCD flash driver
			   ensures that address and size are always aligned to sector size which is multiple of 4 */

			/* write buffer with aligned size */
			int rc = stub_spiflash_write(s_fs.next_write, (uint32_t *)s_fs.out_buf,
				wr_sz, s_encrypt_binary);

			if (rc != STUB_LIB_OK) {
				STUB_LOGE("Failed to write flash (%d)\n", rc);
				return rc;
			}

			/* change counters using unpadding len */
			s_fs.next_write += bytes_in_out_buf;
			s_fs.remaining_uncompressed -= bytes_in_out_buf;
			s_fs.next_out = s_fs.out_buf;
		}
	}

	return ESP_STUB_OK;
}

static int stub_write_data(const uint8_t *buf, uint32_t size)
{
	STUB_LOG_TRACEF("size: %d\n", size);

	return stub_write_aligned_buffer(buf, size);
}

static __maybe_unused int handle_flash_write(va_list ap)
{
	struct esp_flash_stub_flash_write_args *wargs = va_arg(ap, struct esp_flash_stub_flash_write_args *);

	s_encrypt_binary = wargs->options & ESP_STUB_FLASH_ENCRYPT_BINARY ? 1 : 0;

	STUB_LOG_TRACEF("start_addr: %x, total_size: %d, options: %d\n",
		wargs->start_addr, wargs->total_size, wargs->options);

	uint8_t out_buf[ESP_STUB_UNZIP_BUFF_SIZE];
	s_fs.next_write = wargs->start_addr;
	s_fs.remaining_uncompressed = wargs->total_size;
	s_fs.out_buf = out_buf;
	s_fs.next_out = out_buf;

	return stub_apptrace_recv_data(wargs, stub_write_data);
}

/* --- Flash: erase --- */

/**
 * Erase a sector-aligned region of flash
 * Prefer ROM SPIEraseArea when flash is <= 16MB.
 * Large flash mode (flash size > 16MB) falls back to stub_lib_flash_erase_area.
 */
static int stub_flash_erase_region(uint32_t addr, uint32_t size)
{
	int rc = stub_lib_flash_rom_erase_area(addr, size);
	if (rc == STUB_LIB_ERR_NOT_SUPPORTED)
		rc = stub_lib_flash_erase_area(addr, size);
	return rc;
}

static __maybe_unused int handle_flash_erase(va_list ap)
{
	uint32_t start_addr = va_arg(ap, uint32_t);
	uint32_t size = va_arg(ap, uint32_t);

	stub_lib_flash_config_t flash_config;
	stub_lib_flash_get_config(&flash_config);

	start_addr = ALIGN_DOWN(start_addr, flash_config.sector_size);
	size = ALIGN_UP(size, flash_config.sector_size);

	STUB_LOGD("erase flash from 0x%x, sz %d bytes\n", start_addr, size);

	if (start_addr == 0 && size == flash_config.flash_size) {
		STUB_LOGI("Erasing entire flash chip\n");
		return stub_lib_flash_erase_chip();
	}
	return stub_flash_erase_region(start_addr, size);
}

static __maybe_unused int handle_flash_erase_check(va_list ap)
{
	uint32_t start_sec = va_arg(ap, uint32_t);
	uint32_t num_sec = va_arg(ap, uint32_t);
	uint8_t *sec_erased = va_arg(ap, uint8_t *);

	stub_lib_flash_config_t flash_config;
	stub_lib_flash_get_config(&flash_config);

	const uint32_t sector_sz = flash_config.sector_size;
	uint8_t buf[sector_sz];

	STUB_LOGD("erase check start %d, num sectors: %d\n", start_sec, num_sec);

	for (unsigned int i = 0; i < num_sec; i++) {
		sec_erased[i] = 1;
		int rc = stub_lib_mmu_read_flash((start_sec + i) * sector_sz, buf, sizeof(buf));
		if (rc != STUB_LIB_OK) {
			STUB_LOGE("Failed to read flash (%d)!\n", rc);
			return ESP_STUB_FAIL;
		}
		for (unsigned int n = 0; n < sizeof(buf); n++) {
			if (buf[n] != 0xFF) {
				sec_erased[i] = 0;
				break;
			}
		}
	}

	return ESP_STUB_OK;
}

/* --- Flash: mapping --- */

static __maybe_unused int handle_flash_map_get(va_list ap)
{
	uint32_t app_offset_hint = va_arg(ap, uint32_t);
	struct esp_stub_flash_map *out_param = va_arg(ap, struct esp_stub_flash_map *);
	STUB_LOGD("flash mapping, app_offset:0x%x, out_param:0x%x\n", app_offset_hint, out_param);

	return stub_flash_mapping(app_offset_hint, out_param);
}

/* --- Flash: breakpoints --- */

static uint32_t stub_get_inst_buff_size(uint32_t bp_flash_addr, uint8_t insn_sz, uint32_t sector_size)
{
	uint32_t sector_no = ALIGN_DOWN(bp_flash_addr, sector_size) / sector_size;
	uint32_t next_sector_no = ALIGN_DOWN(bp_flash_addr + insn_sz - 1, sector_size) / sector_size;
	return next_sector_no > sector_no ? 2 * sector_size : sector_size;
}

static void stub_cache_pre_flash_write(void)
{
	uint32_t caps = stub_lib_cache_get_caps();
	if (!(caps & STUB_CACHE_CAP_HAS_INVALIDATE_ADDR) && (caps & STUB_CACHE_CAP_SHARED_IDCACHE))
		stub_lib_cache_writeback_all();
}

static void stub_cache_post_flash_write(uint32_t vaddr, uint32_t size)
{
	uint32_t caps = stub_lib_cache_get_caps();
	if (caps & STUB_CACHE_CAP_HAS_INVALIDATE_ADDR)
		stub_lib_cache_invalidate_addr(vaddr, size);
	else
		stub_lib_cache_invalidate_all();
}

/*
 * Possible BP layouts in flash:
 * 1) addr is aligned to 4 bytes (in 1 sector)
 * 2) addr is unaligned to 4 bytes, BP is not crossing sector's boundary (in 1 sector)
 * 3) addr is unaligned to 4 bytes, BP is crossing sector's boundary (in 2 sectors)
 */
static uint8_t stub_flash_set_bp(const uint32_t *bp_addr_pair,
	uint8_t *insn_buf, uint8_t *insn_sect, uint32_t sector_size)
{
	uint32_t bp_flash_addr = bp_addr_pair[0];
	uint32_t bp_virt_addr = bp_addr_pair[1];
	uint32_t inst_buff_size = stub_get_inst_buff_size(bp_flash_addr, (uint8_t)stub_get_max_insn_size(), sector_size);
	uint32_t sect_addr = ALIGN_DOWN(bp_flash_addr, sector_size);
	uint32_t off = bp_flash_addr & (sector_size - 1);
	uint32_t aligned_bp = bp_flash_addr & ~0x3UL;
	uint32_t bp_off = bp_flash_addr - aligned_bp;

	STUB_LOGD("set bp flash addr: %x, virt addr: %x, offset: %d, inst_buff_size: %d\n",
		sect_addr, bp_virt_addr, off, inst_buff_size);

	int rc = stub_lib_mmu_read_flash(sect_addr, insn_sect, inst_buff_size);
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("Failed to read insn sector (%d)!\n", rc);
		return 0;
	}

	uint8_t insn_sz = stub_get_insn_size(&insn_sect[off]);
	memcpy(insn_buf, &insn_sect[off], insn_sz);

	union {
		uint32_t d32;
		uint8_t d8[4];
	} break_insn;
	break_insn.d32 = stub_get_break_insn(insn_sz);
	memcpy(&insn_sect[off], break_insn.d8, insn_sz);

	STUB_LOGD("Set break insn: %x\n", break_insn.d32);

	stub_cache_pre_flash_write();

	stub_lib_cache_stop();

	rc = stub_flash_erase_region(sect_addr, inst_buff_size);
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("Failed to erase insn sector (%d)!\n", rc);
		stub_lib_cache_start();
		return 0;
	}

	rc = stub_spiflash_write(sect_addr, (uint32_t *)insn_sect,
		inst_buff_size, stub_encryption_is_enabled());
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("Failed to write break insn (%d)!\n", rc);
		stub_lib_cache_start();
		return 0;
	}

	stub_cache_post_flash_write(bp_virt_addr, insn_sz);
	stub_lib_cache_start();

	/* Verify the break instruction is written correctly */
	rc = stub_lib_mmu_read_flash(aligned_bp, insn_sect, 8);
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("set_bp: flash readback failed (%d) @ %x\n", rc, bp_flash_addr);
		return 0;
	}
	STUB_LOGD("set_bp: WROTE 0x%x 0x%x [%x %x %x %x %x %x %x %x]\n",
		aligned_bp,
		bp_virt_addr,
		insn_sect[0],
		insn_sect[1],
		insn_sect[2],
		insn_sect[3],
		insn_sect[4],
		insn_sect[5],
		insn_sect[6],
		insn_sect[7]);
	if (memcmp(&insn_sect[bp_off], break_insn.d8, insn_sz) != 0) {
		STUB_LOGE("set_bp: break insn verify failed @ flash %x\n", bp_flash_addr);
		stub_lib_mmu_read_flash(aligned_bp, insn_sect, 8);
		for (unsigned int j = 0; j < 8; j++)
			STUB_LOGE("cache read insn_sect[%d] @ 0x%x: %x\n", j, aligned_bp + j, insn_sect[j]);
		return 0;
	}
	return insn_sz;
}

static __maybe_unused int handle_flash_bp_set(va_list ap)
{
	uint32_t *bp_addr_pairs = va_arg(ap, uint32_t *);
	struct esp_flash_stub_bp_instructions *bp_insts = va_arg(ap, struct esp_flash_stub_bp_instructions *);
	uint8_t *insn_sect = va_arg(ap, uint8_t *);
	uint32_t num_bps = va_arg(ap, uint32_t);

	STUB_LOGD("flash bp set, num_bps: %d\n", num_bps);

	stub_lib_flash_config_t flash_config;
	stub_lib_flash_get_config(&flash_config);

	for (unsigned int i = 0; i < num_bps; ++i) {
		uint8_t rc = stub_flash_set_bp(&bp_addr_pairs[i * 2],
			bp_insts[i].buff, insn_sect, flash_config.sector_size);
		if (rc == 0)
			return ESP_STUB_FAIL;
		bp_insts[i].size = rc;
	}

	return (int)(num_bps * sizeof(struct esp_flash_stub_bp_instructions));
}

static int stub_flash_clear_bp(const uint32_t *bp_addr_pair,
	const uint8_t *insn, uint8_t *insn_sect, uint32_t sector_size)
{
	uint32_t bp_flash_addr = bp_addr_pair[0];
	uint32_t bp_virt_addr = bp_addr_pair[1];
	uint8_t insn_sz = stub_get_insn_size(insn);
	uint32_t inst_buff_size = stub_get_inst_buff_size(bp_flash_addr, insn_sz, sector_size);
	uint32_t sect_addr = ALIGN_DOWN(bp_flash_addr, sector_size);
	uint32_t off = bp_flash_addr & (sector_size - 1);

	STUB_LOGD("clear bp flash addr: %x, virt addr: %x, offset: %d, inst_buff_size: %d\n",
			sect_addr, bp_virt_addr, off, inst_buff_size);

	STUB_LOGD("clear_bp: orig insn: %x %x %x\n", insn[0], insn[1], insn[2]);

	int rc = stub_lib_mmu_read_flash(sect_addr, insn_sect, inst_buff_size);
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("Failed to read insn sector (%d)!\n", rc);
		return ESP_STUB_FAIL;
	}

	memcpy(&insn_sect[off], insn, insn_sz);

	stub_cache_pre_flash_write();

	stub_lib_cache_stop();

	rc = stub_flash_erase_region(sect_addr, inst_buff_size);
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("Failed to erase insn sector (%d)!\n", rc);
		stub_lib_cache_start();
		return ESP_STUB_FAIL;
	}

	rc = stub_spiflash_write(sect_addr, (uint32_t *)insn_sect,
		inst_buff_size, stub_encryption_is_enabled());
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("Failed to restore insn (%d)!\n", rc);
		stub_lib_cache_start();
		return ESP_STUB_FAIL;
	}

	stub_cache_post_flash_write(bp_virt_addr, insn_sz);
	stub_lib_cache_start();

	/* Verify the original instruction is written back correctly */
	uint32_t aligned_bp = bp_flash_addr & ~0x3UL;
	uint32_t bp_off = bp_flash_addr - aligned_bp;
	rc = stub_lib_mmu_read_flash(aligned_bp, insn_sect, 8);
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("clear_bp: flash readback failed (%d) @ %x\n", rc, bp_flash_addr);
		return ESP_STUB_FAIL;
	}
	STUB_LOGD("clear_bp: WROTE 0x%x 0x%x [%x %x %x %x %x %x %x %x]\n",
		aligned_bp,
		bp_virt_addr,
		insn_sect[0],
		insn_sect[1],
		insn_sect[2],
		insn_sect[3],
		insn_sect[4],
		insn_sect[5],
		insn_sect[6],
		insn_sect[7]);
	if (memcmp(&insn_sect[bp_off], insn, insn_sz) != 0) {
		STUB_LOGE("clear_bp: restored insn verify failed @ flash %x\n", bp_flash_addr);
		stub_lib_mmu_read_flash(aligned_bp, insn_sect, 8);
		for (unsigned int j = 0; j < 8; j++)
			STUB_LOGE("cache read insn_sect[%d] @ 0x%x: %x\n", j, aligned_bp + j, insn_sect[j]);
		return ESP_STUB_FAIL;
	}

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_bp_clear(va_list ap)
{
	uint32_t *bp_addr_pairs = va_arg(ap, uint32_t *);
	struct esp_flash_stub_bp_instructions *bp_insts = va_arg(ap, struct esp_flash_stub_bp_instructions *);
	uint8_t *insn_sect = va_arg(ap, uint8_t *);
	uint32_t num_bps = va_arg(ap, uint32_t);

	STUB_LOGD("flash bp clear, num_bps: %d\n", num_bps);

	stub_lib_flash_config_t flash_config;
	stub_lib_flash_get_config(&flash_config);
	uint32_t sector_size = flash_config.sector_size;

	for (uint32_t i = 0; i < num_bps; ++i) {
		int rc = stub_flash_clear_bp(&bp_addr_pairs[i * 2], bp_insts[i].buff, insn_sect, sector_size);
		if (rc != ESP_STUB_OK)
			return rc;
	}

	return ESP_STUB_OK;
}

/* --- Flash: write deflated --- */

static __maybe_unused int stub_write_inflated_data(void *data_buf, uint32_t length)
{
	if (length > s_fs.remaining_uncompressed) {
		/* Trim the final block, as it may have padding beyond
		    the length we are writing */
		length = s_fs.remaining_uncompressed;
	}

	if (length == 0)
		return ESP_STUB_OK;

	/* if this is the last package */
	if (length < ESP_STUB_UNZIP_BUFF_SIZE) {
		/* stub_spiflash_write() expects length to be aligned to 4 bytes.
		   If this is the last package we do not need to care about it here because OpenOCD flash driver
		   ensures that address and size are always aligned to sector size which is multiple of 4 */
		if (s_fs.remaining_uncompressed - length != 0) {
			STUB_LOGE("Unaligned offset! %d-%d\n", length, s_fs.remaining_uncompressed);
			return ESP_STUB_FAIL;
		}
	}

	/* write buffer with aligned size */
	int rc = stub_spiflash_write(s_fs.next_write, (uint32_t *)data_buf, length, s_encrypt_binary);
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("Failed to write flash (%d)\n", rc);
		return ESP_STUB_FAIL;
	}

	return ESP_STUB_OK;
}

static __maybe_unused int stub_run_inflator(const uint8_t *data_buf, uint32_t length)
{
	int status = TINFL_STATUS_NEEDS_MORE_INPUT;

	while (length > 0 && s_fs.remaining_uncompressed > 0) {
		/* input remaining */
		size_t in_bytes = length;
		/* output space remaining */
		size_t out_bytes = (size_t)(s_fs.out_buf + ESP_STUB_UNZIP_BUFF_SIZE - s_fs.next_out);
		mz_uint32 flags = 0;

		if (s_fs.remaining_compressed > length)
			flags |= TINFL_FLAG_HAS_MORE_INPUT;

		status = tinfl_decompress(s_fs.inflator, data_buf, &in_bytes,
			s_fs.out_buf, s_fs.next_out, &out_bytes,
			flags);
		STUB_LOGD("decompress status:%d, in(%d) out(%d)\n", status, in_bytes, out_bytes);

		if (status < TINFL_STATUS_DONE) {
			STUB_LOGE("Failed to decompress data!\n");
			return ESP_STUB_ERR_INFLATE;
		}

		s_fs.remaining_compressed -= in_bytes;
		length -= in_bytes;
		data_buf += in_bytes;
		s_fs.next_out += out_bytes;
		size_t bytes_in_out_buf = (size_t)(s_fs.next_out - s_fs.out_buf);

		if (status == TINFL_STATUS_DONE || bytes_in_out_buf == ESP_STUB_UNZIP_BUFF_SIZE) {
			/* Output buffer full, or done */
			if (stub_write_inflated_data(s_fs.out_buf, bytes_in_out_buf) != ESP_STUB_OK)
				return ESP_STUB_FAIL;

			s_fs.next_write += bytes_in_out_buf;
			s_fs.remaining_uncompressed -= bytes_in_out_buf;
			s_fs.next_out = s_fs.out_buf;
		}
	}	/* while */

	STUB_LOGD("run_inflator rem_uncomp:%d, length:%d\n", s_fs.remaining_uncompressed, length);

	if (s_fs.remaining_uncompressed > 0) {
		if (length == 0)
			return ESP_STUB_OK; /* Need more compressed data */

		STUB_LOGE("Not enough compressed data (%d)\n", s_fs.remaining_uncompressed);
		return ESP_STUB_ERR_NOT_ENOUGH_DATA;
	}

	if (s_fs.remaining_uncompressed == 0 && length > 0) {
		STUB_LOGE("Too much compressed data (%d)\n", length);
		return ESP_STUB_ERR_TOO_MUCH_DATA;
	}

	return ESP_STUB_OK;
}

static __maybe_unused int stub_apptrace_process_recvd_data_deflated(const uint8_t *buf, uint32_t size)
{
	STUB_LOGD("Write zipped data size: %d\n", size);

	return stub_run_inflator(buf, size);
}

static __maybe_unused int handle_flash_write_deflated(va_list ap)
{
	struct esp_flash_stub_flash_write_args *wargs = va_arg(ap, struct esp_flash_stub_flash_write_args *);

	s_encrypt_binary = wargs->options & ESP_STUB_FLASH_ENCRYPT_BINARY ? 1 : 0;

	/* Both of them must be in stack! */
	tinfl_decompressor inflator;
	uint8_t out_buf[ESP_STUB_UNZIP_BUFF_SIZE];

	s_fs.next_write = wargs->start_addr;
	s_fs.remaining_uncompressed = wargs->total_size;
	s_fs.remaining_compressed = wargs->size;
	s_fs.inflator = &inflator;
	s_fs.out_buf = out_buf;
	s_fs.next_out = out_buf;
	tinfl_init(s_fs.inflator);

	return stub_apptrace_recv_data(wargs, stub_apptrace_process_recvd_data_deflated);
}

/* --- Flash: hash & clock --- */

static __maybe_unused int handle_flash_calc_hash(va_list ap)
{
	uint32_t addr = va_arg(ap, uint32_t);
	uint32_t size = va_arg(ap, uint32_t);
	uint8_t *hash = va_arg(ap, uint8_t *);
	uint32_t rd_cnt = 0, rd_sz = 0;
	uint8_t read_buf[ESP_STUB_RDWR_BUFF_SIZE];

	STUB_LOGD("flash calc hash addr: %x, size: %d\n", addr, size);

	stub_lib_sha256_start();

	while (size > 0) {
		rd_sz = MIN(ESP_STUB_RDWR_BUFF_SIZE, size);
		int rc = stub_lib_mmu_read_flash(addr + rd_cnt, read_buf, rd_sz);
		if (rc != STUB_LIB_OK) {
			stub_lib_sha256_finish(NULL);
			STUB_LOGE("Failed to read flash (%d)!\n", rc);
			return ESP_STUB_FAIL;
		}
		stub_lib_sha256_data(read_buf, rd_sz);
		size -= rd_sz;
		rd_cnt += rd_sz;
	}

	stub_lib_sha256_finish(hash);

	STUB_LOGD("hash: %x%x%x...%x%x%x\n",
		hash[0], hash[1], hash[2],
		hash[29], hash[30], hash[31]);

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_clock_configure(va_list ap)
{
	int new_cpu_freq = va_arg(ap, int);

	STUB_LOGD("flash clock configure new_cpu_freq: %d\n", new_cpu_freq);

	if (new_cpu_freq == -1) {
		// set to max frequency
		//stub_lib_clock_init();
	}

	// TODO: return the previous frequency before boost
	// TODO: uart-logger might needs to be reconfigured
	return 0;
}

/* --- Command table --- */

static const struct stub_cmd_handler cmd_handlers[] = {
	#ifdef STUB_CMD_TEST1
	{ESP_STUB_CMD_TEST1, "CMD_TEST1", handle_test1},
	#endif
	#ifdef STUB_CMD_RECV_FROM_HOST
	{ESP_STUB_CMD_RECV_FROM_HOST, "CMD_RECV_FROM_HOST", handle_apptrace_read_from_host},
	#endif
	#ifdef STUB_CMD_SEND_TO_HOST
	{ESP_STUB_CMD_SEND_TO_HOST, "CMD_SEND_TO_HOST", handle_apptrace_write_to_host},
	#endif
	#ifdef STUB_CMD_FLASH_READ
	{ESP_STUB_CMD_FLASH_READ, "CMD_FLASH_READ", handle_flash_read},
	#endif
	#ifdef STUB_CMD_FLASH_WRITE
	{ESP_STUB_CMD_FLASH_WRITE, "CMD_FLASH_WRITE", handle_flash_write},
	#endif
	#ifdef STUB_CMD_FLASH_ERASE
	{ESP_STUB_CMD_FLASH_ERASE, "CMD_FLASH_ERASE", handle_flash_erase},
	#endif
	#ifdef STUB_CMD_FLASH_ERASE_CHECK
	{ESP_STUB_CMD_FLASH_ERASE_CHECK, "CMD_FLASH_ERASE_CHECK", handle_flash_erase_check},
	#endif
	#if defined(STUB_CMD_FLASH_MAP_GET) || defined(STUB_CMD_FLASH_MULTI_COMMAND) || defined(STUB_CMD_FLASH_IDF_BINARY)
	{ESP_STUB_CMD_FLASH_MAP_GET, "CMD_FLASH_MAP_GET", handle_flash_map_get},
	#endif
	#if defined(STUB_CMD_FLASH_BP_SET) || defined(STUB_CMD_FLASH_MULTI_COMMAND) || defined(STUB_CMD_FLASH_IDF_BINARY)
	{ESP_STUB_CMD_FLASH_BP_SET, "CMD_FLASH_BP_SET", handle_flash_bp_set},
	#endif
	#if defined(STUB_CMD_FLASH_BP_CLEAR) || defined(STUB_CMD_FLASH_MULTI_COMMAND) || defined(STUB_CMD_FLASH_IDF_BINARY)
	{ESP_STUB_CMD_FLASH_BP_CLEAR, "CMD_FLASH_BP_CLEAR", handle_flash_bp_clear},
	#endif
	#ifdef STUB_CMD_FLASH_WRITE_DEFLATED
	{ESP_STUB_CMD_FLASH_WRITE_DEFLATED, "CMD_FLASH_WRITE_DEFLATED", handle_flash_write_deflated},
	#endif
	#ifdef STUB_CMD_FLASH_CALC_HASH
	{ESP_STUB_CMD_FLASH_CALC_HASH, "CMD_FLASH_CALC_HASH", handle_flash_calc_hash},
	#endif
	#ifdef STUB_CMD_FLASH_CLOCK_CONFIGURE
	{ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE, "CMD_FLASH_CLOCK_CONFIGURE", handle_flash_clock_configure},
	#endif
	{0, NULL, NULL}
};

/* --- Error mapping & entry point --- */

/**
 * @brief Map error codes for use in OpenOCD
 *
 * Exposes all error codes to OpenOCD without depending on the stub-lib header.
 * - Converts codes from esp-stub-lib/err.h to esp_stub_err.h.
 * - Keeps codes from esp_stub_err.h unchanged.
 */
static int map_stub_error(int ret)
{
	if (ret < STUB_LIB_ERROR_BASE)
		return ret;

	switch (ret) {
	case STUB_LIB_FAIL:
		return ESP_STUB_FAIL;
	case STUB_LIB_OK:
		return ESP_STUB_OK;
	case STUB_LIB_ERR_TIMEOUT:
		return ESP_STUB_ERR_TIMEOUT;
	case STUB_LIB_ERR_UNKNOWN_FLASH_ID:
		return ESP_STUB_ERR_FLASH_ID;
	case STUB_LIB_ERR_FLASH_READ_UNALIGNED:
		return ESP_STUB_ERR_FLASH_READ_UNALIGNED;
	case STUB_LIB_ERR_FLASH_WRITE_UNALIGNED:
		return ESP_STUB_ERR_FLASH_WRITE_UNALIGNED;
	case STUB_LIB_ERR_FLASH_WRITE:
		return ESP_STUB_ERR_FLASH_WRITE;
	case STUB_LIB_ERR_FLASH_READ:
		return ESP_STUB_ERR_FLASH_READ;
	default:
		return ret;
	}
}

int stub_init_flash_and_cache(struct stub_runtime_state *state)
{
	int lib_ret = stub_lib_flash_init_ex(state->flash_state, STUB_LIB_FLASH_ATTACH_IF_NEEDED, true);
	if (lib_ret != STUB_LIB_OK && lib_ret != STUB_LIB_ERR_UNKNOWN_FLASH_ID) {
		print_flash_info();
		return lib_ret;
	}

	stub_lib_cache_init(state->cache_state);

	return ESP_STUB_OK;
}

int stub_deinit_flash_and_cache(const struct stub_runtime_state *state)
{
	stub_lib_flash_deinit(state->flash_state);
	stub_lib_cache_deinit(state->cache_state);

	return ESP_STUB_OK;
}

int stub_main(int cmd, ...)
{
	va_list ap;
	size_t flash_state_sz = stub_lib_flash_state_size();
	size_t cache_state_sz = stub_lib_cache_state_size();
	uint8_t __attribute__((aligned(sizeof(uint32_t)))) flash_state_buf[flash_state_sz ? flash_state_sz : 1];
	uint8_t __attribute__((aligned(sizeof(uint32_t)))) cache_state_buf[cache_state_sz ? cache_state_sz : 1];
	struct stub_runtime_state runtime_state = {
		.flash_state = flash_state_sz ? flash_state_buf : NULL,
		.cache_state = cache_state_sz ? cache_state_buf : NULL,
	};
	int ret = ESP_STUB_ERR_NOT_SUPPORTED;

	/* zero bss */
	for (uint32_t *p = &_bss_start; p < &_bss_end; p++)
		*p = 0;

	va_start(ap, cmd);

	STUB_LOG_INIT(STUB_LIB_LOG_LEVEL);

	STUB_LOGD("Command: 0x%x, flash_encryption: %d\n", cmd, stub_encryption_is_enabled());
	STUB_LOGD("Flash state size: %d cache_state size: %d\n", flash_state_sz, cache_state_sz);

	int lib_ret = stub_init_flash_and_cache(&runtime_state);
	if (lib_ret != ESP_STUB_OK) {
		ret = map_stub_error(lib_ret);
		goto out;
	}

	const struct stub_cmd_handler *handler = cmd_handlers;
	while (handler->handler) {
		if (handler->cmd == cmd) {
			STUB_LOGI("Executing command: %s (0x%x)\n", handler->name, handler->cmd);
			ret = handler->handler(ap);
			if (ret != ESP_STUB_OK)
				goto out;
			break;
		}
		handler++;
	}

	if (!handler->handler)
		ret = ESP_STUB_ERR_NOT_SUPPORTED;

out:
	va_end(ap);

	stub_deinit_flash_and_cache(&runtime_state);

	STUB_LOG_TRACEF("%s ret: %d\n", ret);

	return map_stub_error(ret);
}
