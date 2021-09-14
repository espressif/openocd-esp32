/* Copyright 2021 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. */

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
	if (data_len % 4 != 0)
		return;
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
