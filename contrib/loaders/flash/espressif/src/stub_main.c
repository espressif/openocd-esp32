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
#include "flash_test.h"

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

static __maybe_unused int handle_flash_info(va_list ap)
{
	struct stub_flash_info *info = va_arg(ap, struct stub_flash_info *);
	STUB_LOGD("flash info getting in addr: %x\n", info);

	stub_lib_flash_info_t lib_info;
	stub_lib_flash_get_info(&lib_info);

	static_assert(sizeof(*info) == sizeof(lib_info), "Different stub_flash_info and stub_lib_flash_info");
	memcpy(info, &lib_info, sizeof(*info));

	return ESP_STUB_OK;
}

static __maybe_unused int handle_flash_test(va_list ap)
{
	uint32_t start_addr = va_arg(ap, uint32_t);
	uint32_t data_size = va_arg(ap, uint32_t);
	STUB_LOGD("flash test addr: %x, size: %d\n", start_addr, data_size);

	return stub_flash_test(start_addr, data_size);
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
	#ifdef STUB_CMD_FLASH_INFO
	{ESP_STUB_CMD_FLASH_INFO, "CMD_FLASH_INFO", handle_flash_info},
	#endif
	#ifdef STUB_CMD_FLASH_TEST
	{ESP_STUB_CMD_FLASH_TEST, "CMD_FLASH_TEST", handle_flash_test},
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
		case STUB_LIB_ERR_FLASH_READ_ROM_ERR:
			return STUB_LIB_ERR_FLASH_READ_ROM_ERR;
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
	if (lib_ret != STUB_LIB_OK)
		return map_stub_error(lib_ret);

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
