/* SPDX-License-Identifier: GPL-2.0-or-later */

/* Copyright 2021 Espressif Systems (Shanghai) PTE LTD */

#include <string.h>
#include "stub_rom_chip.h"

static SHA_CTX ctx;

/* this function has the same implementation for ESP32-S2
 * TODO: move to common file */
void stub_sha256_start(void)
{
	/* Enable SHA hardware */
	ets_sha_enable();
	ets_sha_init(&ctx, SHA2_256);
}

void stub_sha256_data(const void *data, size_t data_len)
{
	/* C2 secure boot key field consists of 1 byte of curve identifier and 64 bytes of ECDSA public key.
	* While verifying the signature block, we need to calculate the SHA of this key field which is of 65 bytes.
	* ets_sha_update handles it cleanly so we can safely remove the check:
	* assert(data_len % 4) == 0
	*/
	ets_sha_update(&ctx, data, data_len, false);
}

void stub_sha256_finish(uint8_t *digest)
{
	if (digest == NULL) {
		bzero(&ctx, sizeof(ctx));
		return;
	}
	ets_sha_finish(&ctx, digest);
	ets_sha_disable();
}
