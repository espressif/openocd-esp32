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

/* calculate difference between two struct timeval values */
int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y)
{
	if (x->tv_usec < y->tv_usec) {
		int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
		y->tv_usec -= 1000000 * nsec;
		y->tv_sec += nsec;
	}
	if (x->tv_usec - y->tv_usec > 1000000) {
		int nsec = (x->tv_usec - y->tv_usec) / 1000000;
		y->tv_usec += 1000000 * nsec;
		y->tv_sec -= nsec;
	}

	result->tv_sec = x->tv_sec - y->tv_sec;
	result->tv_usec = x->tv_usec - y->tv_usec;

	/* Return 1 if result is negative. */
	return x->tv_sec < y->tv_sec;
}

int timeval_add_time(struct timeval *result, long sec, long usec)
{
	result->tv_sec += sec;
	result->tv_usec += usec;

	while (result->tv_usec > 1000000) {
		result->tv_usec -= 1000000;
		result->tv_sec++;
	}

	return 0;
}

/* compare two timevals and return -1/0/+1 accordingly */
int timeval_compare(const struct timeval *x, const struct timeval *y)
{
	if (x->tv_sec < y->tv_sec)
		return -1;
	else if (x->tv_sec > y->tv_sec)
		return 1;
	else if (x->tv_usec < y->tv_usec)
		return -1;
	else if (x->tv_usec > y->tv_usec)
		return 1;
	else
		return 0;
}

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
