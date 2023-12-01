/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP chips flasher stub logger definitions                             *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_LOGGER_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_LOGGER_H

#include <stdint.h>

enum stub_log_levels {
	STUB_LOG_LEVEL_NONE,
	STUB_LOG_LEVEL_ERROR,
	STUB_LOG_LEVEL_WARN,
	STUB_LOG_LEVEL_INFO,
	STUB_LOG_LEVEL_DEBUG,
	STUB_LOG_LEVEL_VERBOSE
};

enum stub_log_destination {
	STUB_LOG_DEST_NONE,
	STUB_LOG_DEST_UART,
	STUB_LOG_DEST_USB,
	STUB_LOG_DEST_SRAM,
};

/* #define STUB_LOG_ENABLE 0 */

#define STUB_RESET_LOG_LEVEL STUB_LOG_LEVEL_VERBOSE
#define STUB_RESET_LOG_DEST STUB_LOG_DEST_SRAM

enum stub_log_levels stub_get_log_level(void);
enum stub_log_destination stub_get_log_dest(void);

#define STUB_LOG_BUFF_SIZE 4096
struct stub_log_buffer {
	uint32_t write;
	char buf[STUB_LOG_BUFF_SIZE];
};

#if STUB_LOG_ENABLE == 1

extern int ets_printf(const char *fmt, ...);

extern enum stub_log_levels s_log_level;

void stub_log_init(enum stub_log_levels level, enum stub_log_destination dest);

#define STUB_LOG(level, format, ...)   \
	do { \
		if (s_log_level != STUB_LOG_LEVEL_NONE && s_log_level >= (level)) { \
			ets_printf(format, ## __VA_ARGS__); \
		} \
	} while (0)

#define STUB_LOGE(format, ...)  STUB_LOG(STUB_LOG_LEVEL_ERROR, "STUB_E: " format, ## __VA_ARGS__)
#define STUB_LOGW(format, ...)  STUB_LOG(STUB_LOG_LEVEL_WARN, "STUB_W: "format, ## __VA_ARGS__)
#define STUB_LOGI(format, ...)  STUB_LOG(STUB_LOG_LEVEL_INFO, "STUB_I: "format, ## __VA_ARGS__)
#define STUB_LOGD(format, ...)  STUB_LOG(STUB_LOG_LEVEL_DEBUG, "STUB_D: "format, ## __VA_ARGS__)
#define STUB_LOGV(format, ...)  STUB_LOG(STUB_LOG_LEVEL_VERBOSE, "STUB_V: "format, ## __VA_ARGS__)

#else /* !STUB_LOG_ENABLE */

#define STUB_LOGE(format, ...)  do {} while (0)
#define STUB_LOGW(format, ...)  do {} while (0)
#define STUB_LOGI(format, ...)  do {} while (0)
#define STUB_LOGD(format, ...)  do {} while (0)
#define STUB_LOGV(format, ...)  do {} while (0)

#endif

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_LOGGER_H */
