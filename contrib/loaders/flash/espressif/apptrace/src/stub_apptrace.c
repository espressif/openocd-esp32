// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <string.h>
#include <stdint.h>

#include "stub_logger.h"

#include "stub_flasher.h"
#include "stub_apptrace.h"
#include "apptrace_hw.h"
#include "apptrace_err.h"

static void stub_apptrace_init(void)
{
	apptrace_hw_init();
	apptrace_hw_connect();
}

int stub_apptrace_recv_data(const void *arg, stub_apptrace_recv_cb_t process_cb)
{
	if (!arg || !process_cb) {
		STUB_LOGE("Invalid apptrace read arguments!\n");
		return APPTRACE_ERR_INVALID_ARG;
	}

	struct esp_flash_stub_flash_write_args *wargs = (struct esp_flash_stub_flash_write_args *)arg;
	STUB_LOGD("flash_write: start_addr: %x size: %d", wargs->start_addr, wargs->size);

	/* For RISCV, we need to set the trace memory blocks */
	const uint16_t buf_size = apptrace_hw_get_buf_size();
	__attribute__((unused)) uint8_t stub_trace_mem[buf_size * 2];
	apptrace_hw_set_trace_mem(stub_trace_mem, buf_size * 2);

	stub_apptrace_init();

	apptrace_hw_prep_downlink((uint8_t *)wargs->ring_buf_addr, wargs->ring_buf_size);

	uint32_t total_cnt = 0;
	uint8_t *buf = NULL;
	while (total_cnt < wargs->size) {
		uint32_t wr_sz = wargs->size - total_cnt;
		STUB_LOGD("Req trace down buf %d bytes %d-%d\n", wr_sz, wargs->size, total_cnt);

		buf = apptrace_hw_downlink_get(&wr_sz);
		if (!buf) {
			STUB_LOGE("Failed to get trace down buf!\n");
			return APPTRACE_ERR_FAIL;
		}

		STUB_LOGD("Got trace down buf %d bytes @ 0x%x\n", wr_sz, buf);

		int ret = process_cb(buf, wr_sz);
		/* check the process result */
		if (ret != APPTRACE_ERR_OK) {
			STUB_LOGE("Failed to process trace down buf!\n");
			return ret;
		}

		total_cnt += wr_sz;
	}

	return APPTRACE_ERR_OK;
}

int stub_apptrace_send_data(uint32_t addr, uint32_t size, stub_apptrace_send_cb_t process_cb)
{
	if (!process_cb) {
		STUB_LOGE("Invalid apptrace read arguments!\n");
		return APPTRACE_ERR_INVALID_ARG;
	}

	/* For RISCV, we need to set the trace memory blocks */
	const uint16_t buf_size = apptrace_hw_get_buf_size();
	__attribute__((unused)) uint8_t stub_trace_mem[buf_size * 2];
	apptrace_hw_set_trace_mem(stub_trace_mem, buf_size * 2);

	stub_apptrace_init();

	STUB_LOGI("Start reading %d bytes @ 0x%x\n", size, addr);

	const uint16_t max_user_data_size = apptrace_hw_get_max_user_data_size();

	uint32_t total_cnt = 0;

	while (total_cnt < size) {
		uint32_t rd_sz = size - total_cnt >
			max_user_data_size ? max_user_data_size : size - total_cnt;

		uint8_t *buf = apptrace_hw_uplink_get(rd_sz);
		if (!buf) {
			STUB_LOGE("Failed to get uplink trace buf!\n");
			return APPTRACE_ERR_FAIL;
		}
		STUB_LOGD("Got trace uplink buf %d bytes @ 0x%x\n", rd_sz, buf);

		int ret = process_cb(addr + total_cnt, buf, rd_sz);

		/* regardless of the read result, first free the buffer */
		apptrace_hw_uplink_put(buf);

		/* Now check the process result */
		if (ret != APPTRACE_ERR_OK) {
			STUB_LOGE("Failed to process trace uplink buf!\n");
			return ret;
		}

		total_cnt += rd_sz;

		STUB_LOGD("Flush trace uplink buf %d bytes @ 0x%x [%x %x %x %x %x %x %x %x]\n",
			rd_sz, buf, buf[-4], buf[-3], buf[-2], buf[-1],
			buf[0], buf[1], buf[2], buf[3]);

		ret = apptrace_hw_flush();
		if (ret != APPTRACE_ERR_OK) {
			STUB_LOGE("Failed to flush trace buf!\n");
			return ret;
		}
		STUB_LOGD("Sent trace buf %d bytes @ 0x%x\n", rd_sz, buf);
	}

	STUB_LOGD("Total sent %d/%d bytes\n", total_cnt, size);

	return APPTRACE_ERR_OK;
}
