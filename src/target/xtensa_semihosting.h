/***************************************************************************
 *   Generic Xtensa Semihosting API                                        *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *   Author: Andrei Gramakov <andrei.gramakov@espressif.com>               *
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

#ifndef OPENOCD_XTENSA_SEMIHOSTING_COMMON_H
#define OPENOCD_XTENSA_SEMIHOSTING_COMMON_H

#include "target.h"
#include "command.h"
#include "xtensa.h"
#include "semihosting_common.h"


#define XTENSA_SYSCALL_OP_REG      XT_REG_IDX_A2
#define XTENSA_SYSCALL_RETVAL_REG  XT_REG_IDX_A2
#define XTENSA_SYSCALL_ERRNO_REG   XT_REG_IDX_A3

int xtensa_semihosting_init(struct target *target);

#endif	/*OPENOCD_XTENSA_SEMIHOSTING_COMMON_H*/
