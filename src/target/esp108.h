/***************************************************************************
 *	 Copyright (C) 2015 by Angus Gratton								   *
 *	 gus@projectgus.com													   *
 *																		   *
 *	 This program is free software; you can redistribute it and/or modify  *
 *	 it under the terms of the GNU General Public License as published by  *
 *	 the Free Software Foundation; either version 2 of the License, or	   *
 *	 (at your option) any later version.								   *
 *																		   *
 *	 This program is distributed in the hope that it will be useful,	   *
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *	 GNU General Public License for more details.						   *
 *																		   *
 *	 You should have received a copy of the GNU General Public License	   *
 *	 along with this program; if not, write to the						   *
 *	 Free Software Foundation, Inc.,									   *
 *	 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.		   *
 ***************************************************************************/

#ifndef XTENSA_H
#define XTENSA_H

#include <jtag/jtag.h>
#include "breakpoints.h"

enum xtensa_state {
	XT_NORMAL,
	XT_OCD_RUN,
	XT_OCD_HALT,
};


enum FlashBootstrap {
	FBS_DONTCARE = 0,
	FBS_TMSLOW,
	FBS_TMSHIGH,
};

struct esp108_common {
//	struct jtag_tap *tap;
	enum xtensa_state state;
	struct reg_cache *core_cache;
	struct target *target;
	uint8_t prevpwrstat;
	int resetAsserted;
	int traceActive;

	uint32_t num_brps; /* Number of breakpoints available */
	struct breakpoint **hw_brps;
	uint32_t num_wps; /* Number of watchpoints available */
	struct watchpoint **hw_wps;

	enum FlashBootstrap flashBootstrap; /* 0 - don't care, 1 - TMS low, 2 - TMS high */
};

void esp108_queue_tdi_idle(struct target *target);
void esp108_add_set_ir(struct target *target, uint8_t value);
void esp108_add_dr_scan(struct target *target, int len, const uint8_t *src, uint8_t *dest, tap_state_t endstate);
void esp108_queue_nexus_reg_write(struct target *target, const uint8_t reg, const uint32_t value) ;
void esp108_queue_nexus_reg_read(struct target *target, const uint8_t reg, uint8_t *value) ;

//Small helper function to convert the char arrays that result from a jtag
//call to a well-formatted uint32_t.
static inline uint32_t intfromchars(uint8_t *c) 
{
	return c[0]+(c[1]<<8)+(c[2]<<16)+(c[3]<<24);
}

#endif /* XTENSA_H */
