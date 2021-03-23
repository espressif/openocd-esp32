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

#ifndef ESP32S2_STUB_SHA_H
#define ESP32S2_STUB_SHA_H

void stub_sha256_start(void);
void stub_sha256_data(const void *data, size_t data_len);
void stub_sha256_finish(uint8_t *digest);

#endif
