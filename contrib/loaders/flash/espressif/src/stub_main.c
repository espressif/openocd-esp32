// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

#include <esp-stub-lib/flash.h>
#include <esp-stub-lib/log.h>
#include <esp-stub-lib/bit_utils.h>

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

	return ESP_STUB_ERR_OK;
}

static int stub_apptrace_process_recvd_data(const uint8_t *buf, uint32_t size)
{
	(void)buf;
	(void)size;

	STUB_LOGD("apptrace process data: %x, size: %d\n", buf, size);

	for (uint32_t i = 0; i < MIN(size, 16); i++)
		STUB_LOG("%x ", buf[i]);

	STUB_LOG("\n");

	return ESP_STUB_ERR_OK;
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

	return ESP_STUB_ERR_OK;
}

static __maybe_unused int handle_apptrace_write_to_host(va_list ap)
{
	uint32_t arg1 = va_arg(ap, uint32_t);   //address
	uint32_t arg2 = va_arg(ap, uint32_t);   //size
	//TODO: check address is valid

	STUB_LOGD("apptrace read from host arg ptr: %x\n", arg1);

	return stub_apptrace_send_data(arg1, arg2, stub_apptrace_process_write_data);
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
	{0, NULL, NULL}
};

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

	stub_lib_flash_init(&flash_state);

	STUB_LOGD("Command: %x\n", cmd);

	const struct stub_cmd_handler *handler = cmd_handlers;
	while (handler->handler) {
		if (handler->cmd == cmd) {
			STUB_LOGI("Executing command: %s\n", handler->name);
			ret = handler->handler(ap);
			break;
		}
		handler++;
	}

	if (!handler->handler)
		STUB_LOG("Unknown command!\n");

	va_end(ap);

	if (flash_state)
		stub_lib_flash_deinit(flash_state);

	return ret;
}
