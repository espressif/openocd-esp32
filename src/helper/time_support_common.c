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
 *                                                                         *
 *   Copyright (C) 2026 by Grant Ramsay                                    *
 *   grant.ramsay@hotmail.com                                              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "time_support.h"

#ifdef _WIN32
#include <windows.h>
#endif

/* simple and low overhead fetching of ms counter. Use only
 * the difference between ms counters returned from this fn.
 * Use QueryPerformanceCounter for Windows, else
 * clock_gettime with CLOCK_MONOTONIC_RAW if available, else
 * clock_gettime with CLOCK_MONOTONIC if available, else
 * fallback to gettimeofday (susceptible to NTP adjustments)
 */
int64_t timeval_ms(void)
{
#ifdef _WIN32
	static LARGE_INTEGER frequency;
	LARGE_INTEGER now;
	// frequency is static and only read once on the first call
	if (frequency.QuadPart == 0 && !QueryPerformanceFrequency(&frequency))
		return -1;
	if (!QueryPerformanceCounter(&now))
		return -1;
	return (now.QuadPart * 1000) / frequency.QuadPart;
#elif defined(HAVE_CLOCK_GETTIME)
#ifdef CLOCK_MONOTONIC_RAW
	clockid_t clk_id = CLOCK_MONOTONIC_RAW;
#else
	clockid_t clk_id = CLOCK_MONOTONIC;
#endif
	struct timespec now;
	int retval = clock_gettime(clk_id, &now);
	if (retval < 0)
		return retval;
	return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
#else
	struct timeval now;
	int retval = gettimeofday(&now, NULL);
	if (retval < 0)
		return retval;
	return (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000;
#endif
}
