// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

#include <esp-stub-lib/flash.h>
#include <esp-stub-lib/log.h>
#include <esp-stub-lib/bit_utils.h>
#include <esp-stub-lib/err.h>

#include "esp_stub.h"
#include "stub_apptrace.h"

extern uint32_t _bss_start;
extern uint32_t _bss_end;

#define __maybe_unused __attribute__((unused))

const struct esp_stub_desc __attribute__((section(".stub_desc")))  s_esp_stub_desc = {
	.magic = ESP_STUB_FLASHER_MAGIC_NUM,
	.stub_version = ESP_STUB_FLASHER_VERSION,
	.idf_key = 0x00,
};

struct stub_cmd_handler {
	int cmd;
	const char *name;
	int (*handler)(va_list ap);
};

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

static int stub_apptrace_process_recvd_data(const uint8_t *buf, uint32_t size)
{
	(void)buf;
	(void)size;

	STUB_LOGD("apptrace process data: %x, size: %d\n", buf, size);

	for (uint32_t i = 0; i < MIN(size, 16); i++)
		STUB_LOG("%x ", buf[i]);

	STUB_LOG("\n");

	return ESP_STUB_OK;
}

static __maybe_unused int handle_apptrace_read_from_host(va_list ap)
{
	uint32_t arg1 = va_arg(ap, uint32_t);

	//TODO: check address is valid

	STUB_LOGD("apptrace read from host arg ptr: %x\n", arg1);

	return stub_apptrace_recv_data((void *)arg1, stub_apptrace_process_recvd_data);
}

static  __maybe_unused int stub_apptrace_process_write_data(uint32_t addr, uint8_t *buf, uint32_t size)
{
	(void)addr;

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

static __maybe_unused int handle_flash_read(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	uint32_t __maybe_unused data_size = va_arg(ap, uint32_t);
	STUB_LOGD("flash read addr: %x, size: %d\n", start_addr, data_size);

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_write(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	uint32_t __maybe_unused data_size = va_arg(ap, uint32_t);
	STUB_LOGD("flash write addr: %x, size: %d\n", start_addr, data_size);

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_erase(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	STUB_LOGD("flash erase addr: %x\n", start_addr);

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_erase_check(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	STUB_LOGD("flash erase check addr: %x\n", start_addr);

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_map_get(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	STUB_LOGD("flash map get addr: %x\n", start_addr);

	return ESP_STUB_OK;
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

static __maybe_unused int handle_flash_write_deflated(va_list ap)
{
	uint32_t __maybe_unused start_addr = va_arg(ap, uint32_t);
	uint32_t __maybe_unused data_size = va_arg(ap, uint32_t);
	STUB_LOGD("flash write deflated addr: %x, size: %d\n", start_addr, data_size);

	return ESP_STUB_OK;
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

/* Expose all error codes to OOCD without stub-lib header dependency */
static int map_stub_error(int ret)
{
	switch (ret) {
		case STUB_LIB_FAIL:
			return ESP_STUB_FAIL;
		case STUB_LIB_OK:
			return ESP_STUB_OK;
		case STUB_LIB_ERR_UNKNOWN_FLASH_ID:
			return ESP_STUB_ERR_FLASH_SIZE;
		case STUB_LIB_ERR_FLASH_READ_UNALIGNED:
			return ESP_STUB_ERR_FLASH_READ_UNALIGNED;
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

	STUB_LOG_INIT();

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
