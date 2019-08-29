/***************************************************************************
 *   ESP xtensa chips flasher stub internal definitions                    *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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
#ifndef ESP_FLASHER_STUB_INT_H
#define ESP_FLASHER_STUB_INT_H

#include "rom/ets_sys.h"

#define STUB_LOG_NONE           0
#define STUB_LOG_ERROR          1
#define STUB_LOG_WARN           2
#define STUB_LOG_INFO           3
#define STUB_LOG_DEBUG          4
#define STUB_LOG_VERBOSE        5

#define STUB_LOG_LOCAL_LEVEL  STUB_LOG_NONE

#define STUB_LOG(level, format, ...)   \
	do { \
		if (STUB_LOG_LOCAL_LEVEL >= level) { \
			ets_printf(format, ## __VA_ARGS__); \
		} \
	} while (0)

#define STUB_LOGE(format, ...)  STUB_LOG(STUB_LOG_ERROR, "STUB_E: " format, ## __VA_ARGS__)
#define STUB_LOGW(format, ...)  STUB_LOG(STUB_LOG_WARN, "STUB_W: "format, ## __VA_ARGS__)
#define STUB_LOGI(format, ...)  STUB_LOG(STUB_LOG_INFO, "STUB_I: "format, ## __VA_ARGS__)
#define STUB_LOGD(format, ...)  STUB_LOG(STUB_LOG_DEBUG, "STUB_D: "format, ## __VA_ARGS__)
#define STUB_LOGV(format, ...)  STUB_LOG(STUB_LOG_VERBOSE, "STUB_V: "format, ## __VA_ARGS__)

#endif	/*ESP_FLASHER_STUB_INT_H */
