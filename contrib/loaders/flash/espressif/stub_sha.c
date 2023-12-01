// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP chips flasher stub sha calculation                                *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <sdkconfig.h>

#if !CONFIG_IDF_TARGET_ESP32

#include <string.h>
#include <stdint.h>

#include <stub_flasher_chip.h>

static SHA_CTX ctx;

void stub_sha256_start(void)
{
	/* Enable SHA hardware */
	ets_sha_enable();
	ets_sha_init(&ctx, SHA2_256);
}

void stub_sha256_data(const void *data, size_t data_len)
{
	if (data_len % 4 != 0)
		return;
	ets_sha_update(&ctx, data, data_len, false);
}

void stub_sha256_finish(uint8_t *digest)
{
	if (!digest) {
		bzero(&ctx, sizeof(ctx));
		return;
	}
	ets_sha_finish(&ctx, digest);
	ets_sha_disable();
}

#endif /* !CONFIG_IDF_TARGET_ESP32 */
