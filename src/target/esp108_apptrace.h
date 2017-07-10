/***************************************************************************
 *   ESP108 application tracing module for OpenOCD                         *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   <alexey@espressif.com>                                                *
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


#ifndef XTENSA_ESP108_APPTRACE_H
#define XTENSA_ESP108_APPTRACE_H

#include "command.h"

int esp_cmd_apptrace_generic(struct target *target, int sys_view, const char **argv, int argc);
__COMMAND_HANDLER(esp108_cmd_apptrace);
__COMMAND_HANDLER(esp108_cmd_sysview);

#endif // XTENSA_ESP108_APPTRACE_H
