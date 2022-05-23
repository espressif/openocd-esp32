/***************************************************************************
 *   Semihosting API for Espressif chips                                   *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_SEMIHOSTING_H
#define OPENOCD_TARGET_ESP_SEMIHOSTING_H

/* Legacy syscalls */
#define ESP_SYS_DRV_INFO_LEGACY                     0xE0

/* syscalls compatible to ARM standard */
#define ESP_SEMIHOSTING_SYS_DRV_INFO                0x100
#define ESP_SEMIHOSTING_SYS_APPTRACE_INIT           0x101
#define ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT        0x102
#define ESP_SEMIHOSTING_SYS_BREAKPOINT_SET          0x103
#define ESP_SEMIHOSTING_SYS_WATCHPOINT_SET          0x104
#define ESP_SEMIHOSTING_SYS_SEEK                    0x105	/* custom lseek with whence */
/* not implemented yet */
#define ESP_SEMIHOSTING_SYS_MKDIR                   0x106
#define ESP_SEMIHOSTING_SYS_OPENDIR                 0x107
#define ESP_SEMIHOSTING_SYS_READDIR                 0x108
#define ESP_SEMIHOSTING_SYS_READDIR_R               0x109
#define ESP_SEMIHOSTING_SYS_SEEKDIR                 0x10A
#define ESP_SEMIHOSTING_SYS_TELLDIR                 0x10B
#define ESP_SEMIHOSTING_SYS_CLOSEDIR                0x10C
#define ESP_SEMIHOSTING_SYS_RMDIR                   0x10D
#define ESP_SEMIHOSTING_SYS_ACCESS                  0x10E
#define ESP_SEMIHOSTING_SYS_TRUNCATE                0x10F
#define ESP_SEMIHOSTING_SYS_UTIME                   0x110
#define ESP_SEMIHOSTING_SYS_FSTAT                   0x111
#define ESP_SEMIHOSTING_SYS_STAT                    0x112
#define ESP_SEMIHOSTING_SYS_FSYNC                   0x113
#define ESP_SEMIHOSTING_SYS_LINK                    0x114
#define ESP_SEMIHOSTING_SYS_UNLINK                  0x115

int esp_semihosting_common(struct target *target);
int esp_semihosting_basedir_command(struct command_invocation *cmd);

#endif	/* OPENOCD_TARGET_ESP_SEMIHOSTING_H */
