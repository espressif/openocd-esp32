/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#pragma once

#include <stdint.h>

typedef struct stub_flash_info {
    uint32_t id;
    uint32_t size;
    uint32_t block_size;
    uint32_t sector_size;
    uint32_t page_size;
    uint32_t mode; // SPI, Octal, Dual, Quad
    uint32_t encrypted;
} stub_lib_flash_info_t;

void stub_lib_flash_init(void **state);
void stub_lib_flash_deinit(const void *state);
void stub_lib_flash_get_info(stub_lib_flash_info_t *info);
int stub_lib_flash_read_buff(uint32_t addr, void *buffer, uint32_t size);
int stub_lib_flash_write_buff(uint32_t addr, const void *buffer, uint32_t size, int encrypt);
int stub_lib_flash_erase_area(uint32_t addr, uint32_t size);
int stub_lib_flash_erase_sector(uint32_t addr);
int stub_lib_flash_erase_block(uint32_t addr);
int stub_lib_flash_erase_chip(void);
