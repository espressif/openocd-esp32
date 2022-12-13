/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP chips flasher stub logger definitions                             *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include "stub_logger.h"
#include "stub_flasher_int.h"

enum stub_log_levels s_log_level = STUB_LOG_LEVEL_NONE;
static enum stub_log_destination s_log_dest = STUB_LOG_DEST_SRAM;

#if STUB_LOG_ENABLE == 1

static struct stub_log_buffer s_stub_log_buff;

static void stub_log_buffer_write(char c)
{
	s_stub_log_buff.buf[s_stub_log_buff.write] = c;
	s_stub_log_buff.write = (s_stub_log_buff.write + 1) & (STUB_LOG_BUFF_SIZE - 1);
}

void stub_log_init(enum stub_log_levels level, enum stub_log_destination dest)
{
	if (level != STUB_LOG_LEVEL_NONE) {
		if (dest == STUB_LOG_DEST_UART || dest == STUB_LOG_DEST_USB) {
			stub_uart_console_configure(dest);
		} else if (dest == STUB_LOG_DEST_SRAM) {
			ets_install_putc1(stub_log_buffer_write);
			s_stub_log_buff.write = 0;
		} else {
			level = STUB_LOG_LEVEL_NONE;
		}
	}

	s_log_level = level;
	s_log_dest = level == STUB_LOG_LEVEL_NONE ? STUB_LOG_DEST_NONE : dest;
}

#endif

inline enum stub_log_levels stub_get_log_level(void)
{
	return s_log_level;
}

inline enum stub_log_destination stub_get_log_dest(void)
{
	return s_log_dest;
}
