// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

#include <esp-stub-lib/flash.h>
#include <esp-stub-lib/log.h>
#include <esp-stub-lib/bit_utils.h>
#include <esp-stub-lib/err.h>
#include <esp-stub-lib/miniz.h>
#include <esp-stub-lib/security.h>

#include "esp_stub.h"
#include "stub_apptrace.h"
#include "flash_mapping.h"

extern uint32_t _bss_start;
extern uint32_t _bss_end;

#define __maybe_unused __attribute__((unused))

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
	/* */
	uint8_t *next_out;
} s_fs;

static uint32_t s_encrypt_binary;

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

	return ESP_STUB_OK;
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

	//TODO: check address is valid

	STUB_LOGD("apptrace read from host arg ptr: %x\n", (uint32_t)arg1);

	return stub_apptrace_recv_data(arg1, stub_apptrace_process_recvd_data);
}

static  int stub_apptrace_process_write_data(uint32_t __maybe_unused addr, uint8_t *buf, uint32_t size)
{
	STUB_LOGD("apptrace write to host addr: %x, size: %d\n", addr, size);

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
	//TODO: check address is valid

	STUB_LOGD("apptrace read from host arg ptr: %x\n", arg1);

	return stub_apptrace_send_data(arg1, arg2, stub_apptrace_process_write_data);
}

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

/* All flash commands should be implemented here */

static int stub_read_data(uint32_t addr, uint8_t *buf, uint32_t size)
{
	STUB_LOG_TRACEF("addr: %x, size: %d\n", addr, size);

	return stub_lib_flash_read_buff(addr, (uint32_t *)buf, size);
}

static __maybe_unused int handle_flash_read(va_list ap)
{
	uint32_t addr = va_arg(ap, uint32_t);
	uint32_t size = va_arg(ap, uint32_t);

	STUB_LOG_TRACEF("start addr: %x, size: %d\n", addr, size);

	return stub_apptrace_send_data(addr, size, stub_read_data);
}

static __maybe_unused bool stub_encryption_is_enabled(void)
{
	static bool first;
	static bool encryption_enabled;

	if (!first) {
		first = true;
		encryption_enabled = stub_lib_security_flash_is_encrypted();
	}

	STUB_LOGD("Flash encryption enabled: %d\n", encryption_enabled);

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

	//TODO: measure the write time
	int rc = stub_lib_flash_write_buff(spi_flash_addr, data, len, encrypt);

	STUB_LOGD("Write %sflash @ 0x%x sz %d\n",
		encrypt ? "encrypted-" : "",
		spi_flash_addr,
		len);

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
	return stub_lib_flash_erase_area(start_addr, size);
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
		int rc = stub_lib_flash_read_buff((start_sec + i) * sector_sz, buf, sizeof(buf));
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

static __maybe_unused int handle_flash_map_get(va_list ap)
{
	uint32_t app_offset_hint = va_arg(ap, uint32_t);
	struct esp_stub_flash_map *out_param = va_arg(ap, struct esp_stub_flash_map *);
	STUB_LOGD("flash mapping, app_offset:0x%x, out_param:0x%x\n", app_offset_hint, out_param);

	return stub_flash_mapping(app_offset_hint, out_param);
}

static __maybe_unused int handle_flash_bp_set(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	STUB_LOGD("flash bp set addr: %x\n", start_addr);

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_bp_clear(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	STUB_LOGD("flash bp clear addr: %x\n", start_addr);

	return ESP_STUB_OK;
}

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

static __maybe_unused int stub_run_inflator(const void *data_buf, uint32_t length)
{
	int status = TINFL_STATUS_NEEDS_MORE_INPUT;

	while (length > 0 && s_fs.remaining_uncompressed > 0 && status > TINFL_STATUS_DONE) {
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
		STUB_LOGV("tinfl_decompress in(%d) out(%d)\n", in_bytes, out_bytes);

		s_fs.remaining_compressed -= in_bytes;
		length -= in_bytes;
		data_buf += in_bytes;
		s_fs.next_out += out_bytes;
		size_t bytes_in_out_buf = (size_t)(s_fs.next_out - s_fs.out_buf);

		if (status <= TINFL_STATUS_DONE ||
			bytes_in_out_buf == ESP_STUB_UNZIP_BUFF_SIZE) {
			/* Output buffer full, or done */
			if (stub_write_inflated_data(s_fs.out_buf, bytes_in_out_buf) != ESP_STUB_OK)
				return ESP_STUB_FAIL;

			s_fs.next_write += bytes_in_out_buf;
			s_fs.remaining_uncompressed -= bytes_in_out_buf;
			s_fs.next_out = s_fs.out_buf;
		}
	}	/* while */

	if (status < TINFL_STATUS_DONE) {
		STUB_LOGE("Failed to inflate data (%d)\n", status);
		return ESP_STUB_ERR_INFLATE;
	}
	if (status == TINFL_STATUS_DONE && s_fs.remaining_uncompressed > 0) {
		STUB_LOGE("Not enough compressed data (%d)\n", s_fs.remaining_uncompressed);
		return ESP_STUB_ERR_NOT_ENOUGH_DATA;
	}
	if (status != TINFL_STATUS_DONE && s_fs.remaining_uncompressed == 0) {
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

static __maybe_unused int handle_flash_calc_hash(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	uint32_t __maybe_unused data_size = va_arg(ap, uint32_t);
	STUB_LOGD("flash calc hash addr: %x, size: %d\n", start_addr, data_size);

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_clock_configure(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	STUB_LOGD("flash clock configure addr: %x\n", start_addr);

	return ESP_STUB_OK;
}

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

/**
 * @brief Map error codes for use in OpenOCD
 *
 * Exposes all error codes to OpenOCD without depending on the stub-lib header.
 * - Converts codes from esp-stub-lib/err.h to esp_stub_err.h.
 * - Keeps codes from esp_stub_err.h unchanged.
 */
static int map_stub_error(int ret)
{
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

int stub_main(int cmd, ...)
{
	va_list ap;
	void *flash_state = NULL;
	int ret = ESP_STUB_ERR_NOT_SUPPORTED;

	/* zero bss */
	for (uint32_t *p = &_bss_start; p < &_bss_end; p++)
		*p = 0;

	va_start(ap, cmd);

	STUB_LOG_INIT(STUB_LIB_LOG_LEVEL);

	STUB_LOGD("Command: 0x%x\n", cmd);

	int lib_ret = stub_lib_flash_init(&flash_state);
	if (lib_ret != STUB_LIB_OK) {
		print_flash_info();
		return map_stub_error(lib_ret);
	}

	const struct stub_cmd_handler *handler = cmd_handlers;
	while (handler->handler) {
		if (handler->cmd == cmd) {
			STUB_LOGI("Executing command: %s (0x%x)\n", handler->name, handler->cmd);
			ret = handler->handler(ap);
			if (ret != ESP_STUB_OK)
				goto flash_va_end;
			break;
		}
		handler++;
	}

	if (!handler->handler)
		ret = ESP_STUB_ERR_NOT_SUPPORTED;

flash_va_end:
	va_end(ap);

	if (flash_state)
		stub_lib_flash_deinit(flash_state);

	return map_stub_error(ret);
}
