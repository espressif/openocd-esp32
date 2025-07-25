/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-H4 flasher stub definitions                                     *
 *   Copyright (C) 2025 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32H4_STUB_FLASHER_CHIP_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32H4_STUB_FLASHER_CHIP_H

#include "esp32h4/rom/spi_flash.h"
#include "esp32h4/rom/sha.h"
#include "miniz.h"
#include <soc/spi_mem_reg.h>

/** SPI1_MEM_C_FLASH_HPM : R/W/SC; bitpos: [19]; default: 0;
 *  Drive Flash into high performance mode.  The bit will be cleared once the operation
 *  done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_HPM    (BIT(19))
#define SPI1_MEM_C_FLASH_HPM_M  (SPI1_MEM_C_FLASH_HPM_V << SPI1_MEM_C_FLASH_HPM_S)
#define SPI1_MEM_C_FLASH_HPM_V  0x00000001U
#define SPI1_MEM_C_FLASH_HPM_S  19
/** SPI1_MEM_C_FLASH_RES : R/W/SC; bitpos: [20]; default: 0;
 *  This bit combined with reg_resandres bit releases Flash from the power-down state
 *  or high performance mode and obtains the devices ID. The bit will be cleared once
 *  the operation done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_RES    (BIT(20))
#define SPI1_MEM_C_FLASH_RES_M  (SPI1_MEM_C_FLASH_RES_V << SPI1_MEM_C_FLASH_RES_S)
#define SPI1_MEM_C_FLASH_RES_V  0x00000001U
#define SPI1_MEM_C_FLASH_RES_S  20
/** SPI1_MEM_C_FLASH_DP : R/W/SC; bitpos: [21]; default: 0;
 *  Drive Flash into power down.  An operation will be triggered when the bit is set.
 *  The bit will be cleared once the operation done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_DP    (BIT(21))
#define SPI1_MEM_C_FLASH_DP_M  (SPI1_MEM_C_FLASH_DP_V << SPI1_MEM_C_FLASH_DP_S)
#define SPI1_MEM_C_FLASH_DP_V  0x00000001U
#define SPI1_MEM_C_FLASH_DP_S  21
/** SPI1_MEM_C_FLASH_CE : R/W/SC; bitpos: [22]; default: 0;
 *  Chip erase enable. Chip erase operation will be triggered when the bit is set. The
 *  bit will be cleared once the operation done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_CE    (BIT(22))
#define SPI1_MEM_C_FLASH_CE_M  (SPI1_MEM_C_FLASH_CE_V << SPI1_MEM_C_FLASH_CE_S)
#define SPI1_MEM_C_FLASH_CE_V  0x00000001U
#define SPI1_MEM_C_FLASH_CE_S  22
/** SPI1_MEM_C_FLASH_BE : R/W/SC; bitpos: [23]; default: 0;
 *  Block erase enable(32KB) .  Block erase operation will be triggered when the bit is
 *  set. The bit will be cleared once the operation done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_BE    (BIT(23))
#define SPI1_MEM_C_FLASH_BE_M  (SPI1_MEM_C_FLASH_BE_V << SPI1_MEM_C_FLASH_BE_S)
#define SPI1_MEM_C_FLASH_BE_V  0x00000001U
#define SPI1_MEM_C_FLASH_BE_S  23
/** SPI1_MEM_C_FLASH_SE : R/W/SC; bitpos: [24]; default: 0;
 *  Sector erase enable(4KB). Sector erase operation will be triggered when the bit is
 *  set. The bit will be cleared once the operation done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_SE    (BIT(24))
#define SPI1_MEM_C_FLASH_SE_M  (SPI1_MEM_C_FLASH_SE_V << SPI1_MEM_C_FLASH_SE_S)
#define SPI1_MEM_C_FLASH_SE_V  0x00000001U
#define SPI1_MEM_C_FLASH_SE_S  24
/** SPI1_MEM_C_FLASH_PP : R/W/SC; bitpos: [25]; default: 0;
 *  Page program enable(1 byte ~256 bytes data to be programmed). Page program
 *  operation  will be triggered when the bit is set. The bit will be cleared once the
 *  operation done .1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_PP    (BIT(25))
#define SPI1_MEM_C_FLASH_PP_M  (SPI1_MEM_C_FLASH_PP_V << SPI1_MEM_C_FLASH_PP_S)
#define SPI1_MEM_C_FLASH_PP_V  0x00000001U
#define SPI1_MEM_C_FLASH_PP_S  25
/** SPI1_MEM_C_FLASH_WRSR : R/W/SC; bitpos: [26]; default: 0;
 *  Write status register enable.   Write status operation  will be triggered when the
 *  bit is set. The bit will be cleared once the operation done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_WRSR    (BIT(26))
#define SPI1_MEM_C_FLASH_WRSR_M  (SPI1_MEM_C_FLASH_WRSR_V << SPI1_MEM_C_FLASH_WRSR_S)
#define SPI1_MEM_C_FLASH_WRSR_V  0x00000001U
#define SPI1_MEM_C_FLASH_WRSR_S  26
/** SPI1_MEM_C_FLASH_RDSR : R/W/SC; bitpos: [27]; default: 0;
 *  Read status register-1.  Read status operation will be triggered when the bit is
 *  set. The bit will be cleared once the operation done.1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_RDSR    (BIT(27))
#define SPI1_MEM_C_FLASH_RDSR_M  (SPI1_MEM_C_FLASH_RDSR_V << SPI1_MEM_C_FLASH_RDSR_S)
#define SPI1_MEM_C_FLASH_RDSR_V  0x00000001U
#define SPI1_MEM_C_FLASH_RDSR_S  27
/** SPI1_MEM_C_FLASH_RDID : R/W/SC; bitpos: [28]; default: 0;
 *  Read JEDEC ID . Read ID command will be sent when the bit is set. The bit will be
 *  cleared once the operation done. 1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_RDID    (BIT(28))
#define SPI1_MEM_C_FLASH_RDID_M  (SPI1_MEM_C_FLASH_RDID_V << SPI1_MEM_C_FLASH_RDID_S)
#define SPI1_MEM_C_FLASH_RDID_V  0x00000001U
#define SPI1_MEM_C_FLASH_RDID_S  28
/** SPI1_MEM_C_FLASH_WRDI : R/W/SC; bitpos: [29]; default: 0;
 *  Write flash disable. Write disable command will be sent when the bit is set. The
 *  bit will be cleared once the operation done. 1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_WRDI    (BIT(29))
#define SPI1_MEM_C_FLASH_WRDI_M  (SPI1_MEM_C_FLASH_WRDI_V << SPI1_MEM_C_FLASH_WRDI_S)
#define SPI1_MEM_C_FLASH_WRDI_V  0x00000001U
#define SPI1_MEM_C_FLASH_WRDI_S  29
/** SPI1_MEM_C_FLASH_WREN : R/W/SC; bitpos: [30]; default: 0;
 *  Write flash enable.  Write enable command will be sent when the bit is set. The bit
 *  will be cleared once the operation done. 1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_WREN    (BIT(30))
#define SPI1_MEM_C_FLASH_WREN_M  (SPI1_MEM_C_FLASH_WREN_V << SPI1_MEM_C_FLASH_WREN_S)
#define SPI1_MEM_C_FLASH_WREN_V  0x00000001U
#define SPI1_MEM_C_FLASH_WREN_S  30
/** SPI1_MEM_C_FLASH_READ : R/W/SC; bitpos: [31]; default: 0;
 *  Read flash enable. Read flash operation will be triggered when the bit is set. The
 *  bit will be cleared once the operation done. 1: enable 0: disable.
 */
#define SPI1_MEM_C_FLASH_READ    (BIT(31))
#define SPI1_MEM_C_FLASH_READ_M  (SPI1_MEM_C_FLASH_READ_V << SPI1_MEM_C_FLASH_READ_S)
#define SPI1_MEM_C_FLASH_READ_V  0x00000001U
#define SPI1_MEM_C_FLASH_READ_S  31


#include "stub_riscv_common.h"

#define SPI_MEM_FLASH_RDID SPI1_MEM_C_FLASH_RDID

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32H4_STUB_FLASHER_CHIP_H */
