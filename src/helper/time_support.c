// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>

#include "time_support.h"

int duration_start(struct duration *duration)
{
	int64_t now = timeval_ms();

	if (now < 0)
		return ERROR_FAIL;

	duration->start_ms = now;

	return ERROR_OK;
}

int duration_measure(struct duration *duration)
{
	int64_t now = timeval_ms();

	if (now < 0)
		return ERROR_FAIL;

	duration->elapsed_ms = now - duration->start_ms;

	return ERROR_OK;
}

float duration_elapsed(const struct duration *duration)
{
	return ((float)duration->elapsed_ms) / 1000;
}

float duration_kbps(const struct duration *duration, size_t count)
{
	int64_t elapsed_ms = duration->elapsed_ms;

	if (elapsed_ms == 0)
		elapsed_ms = 1;

	return 1000 * count / (1024 * (float)elapsed_ms);
}
