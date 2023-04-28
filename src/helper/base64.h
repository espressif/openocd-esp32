/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Base64 encoding/decoding (RFC1341)
 * Copyright (c) 2005, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef OPENOCD_HELPER_BASE64_H
#define OPENOCD_HELPER_BASE64_H

unsigned char *base64_encode(const unsigned char *src, size_t len,
			      size_t *out_len);
unsigned char *base64_decode(const unsigned char *src, size_t len,
			      size_t *out_len);

#endif /* OPENOCD_HELPER_BASE64_H */
