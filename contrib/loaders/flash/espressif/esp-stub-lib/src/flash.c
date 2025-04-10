/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <flash.h>
#include <target/flash.h>

void stub_lib_flash_init(void **state)
{
    stub_target_flash_init(state);
}

void stub_lib_flash_deinit(const void *state)
{
    stub_target_flash_deinit(state);
}

void stub_lib_flash_get_info(stub_lib_flash_info_t *info)
{
    (void)info;
    // TODO: implement

}
