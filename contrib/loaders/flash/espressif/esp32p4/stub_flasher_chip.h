/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-P4 flasher stub definitions                                     *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32P4_STUB_FLASHER_CHIP_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32P4_STUB_FLASHER_CHIP_H

#include "esp32p4/rom/spi_flash.h"
#include "esp32p4/rom/sha.h"
#include "miniz.h"
#include <soc/spi_mem_reg.h>

#include "stub_riscv_common.h"

#define SPI_MEM_FLASH_RDID SPI1_MEM_C_FLASH_RDID

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32P4_STUB_FLASHER_CHIP_H */
