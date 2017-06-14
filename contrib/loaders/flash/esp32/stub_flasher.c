/*
 * Copyright (c) 2016 Cesanta Software Limited & Espressif Systems (Shanghai) PTE LTD
 * All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
 * Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Spiffy flasher. Implements strong checksums (MD5) and can use higher
 * baud rates. Actual max baud rate will differ from device to device,
 * but 921K seems to be common.
 *
 * SLIP protocol is used for communication.
 * First packet is a single byte - command number.
 * After that, a packet with a variable number of 32-bit (LE) arguments,
 * depending on command.
 *
 * Then command produces variable number of packets of output, but first
 * packet of length 1 is the response code: 0 for success, non-zero - error.
 *
 * See individual command description below.
 */

#include <stdarg.h>
#include <string.h>
#include "soc/dport_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "rom/ets_sys.h"
#include "rom/spi_flash.h"

#define SPI_FLASH_SEC_SIZE  4096    /**< SPI Flash sector size */

#define STUB_ERR_OK             0
#define STUB_ERR_FAIL           (-1)
#define STUB_ERR_NOT_SUPPORTED  (-2)

#define STUB_CMD_TEST         0
#define STUB_CMD_FLASH_READ   1
#define STUB_CMD_FLASH_WRITE  2
#define STUB_CMD_FLASH_ERASE  3
#define STUB_CMD_FLASH_TEST   4

extern uint32_t _bss_start;
extern uint32_t _bss_end;

volatile int g_count_off = 90;

//volatile uint8_t g_stub_stack[STUB_STACK_SZ] __attribute__((section(".stub_stack")));

/**
 * The following two functions are replacements for Cache_Read_Disable and Cache_Read_Enable
 * function in ROM. They are used to work around a bug where Cache_Read_Disable requires a call to
 * Cache_Flush before Cache_Read_Enable, even if cached data was not modified.
 */
static const uint32_t cache_mask  = DPORT_APP_CACHE_MASK_OPSDRAM | DPORT_APP_CACHE_MASK_DROM0 |
        DPORT_APP_CACHE_MASK_DRAM1 | DPORT_APP_CACHE_MASK_IROM0 |
        DPORT_APP_CACHE_MASK_IRAM1 | DPORT_APP_CACHE_MASK_IRAM0;

static void stub_spi_flash_disable_cache(uint32_t cpuid, uint32_t* saved_state)
{
    uint32_t ret = 0;
    if (cpuid == 0) {
        ret |= GET_PERI_REG_BITS2(DPORT_PRO_CACHE_CTRL1_REG, cache_mask, 0);
        while (GET_PERI_REG_BITS2(DPORT_PRO_DCACHE_DBUG0_REG, DPORT_PRO_CACHE_STATE, DPORT_PRO_CACHE_STATE_S) != 1) {
            ;
        }
        SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 0, DPORT_PRO_CACHE_ENABLE_S);
    } else {
        ret |= GET_PERI_REG_BITS2(DPORT_APP_CACHE_CTRL1_REG, cache_mask, 0);
        while (GET_PERI_REG_BITS2(DPORT_APP_DCACHE_DBUG0_REG, DPORT_APP_CACHE_STATE, DPORT_APP_CACHE_STATE_S) != 1) {
            ;
        }
        SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 0, DPORT_APP_CACHE_ENABLE_S);
    }
    *saved_state = ret;
}

static void stub_spi_flash_restore_cache(uint32_t cpuid, uint32_t saved_state)
{
    if (cpuid == 0) {
        SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 1, DPORT_PRO_CACHE_ENABLE_S);
        SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL1_REG, cache_mask, saved_state, 0);
    } else {
        SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 1, DPORT_APP_CACHE_ENABLE_S);
        SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL1_REG, cache_mask, saved_state, 0);
    }
}

static inline uint32_t stub_get_coreid() {
    int id;
    __asm__ volatile(
        "rsr.prid %0\n"
        " extui %0,%0,13,1"
        :"=r"(id));
    return id;
}

static int stub_test(int cnt)
{
  g_count_off += 16;
  return cnt + g_count_off;
}

static int stub_flash_test(void)
{
  int ret = STUB_ERR_OK;
  uint8_t buf[32] = {9, 1, 2, 3, 4, 5, 6, 8};
  uint32_t flash_addr = 0x1d4000;

  esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_sector(flash_addr/SPI_FLASH_SEC_SIZE);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    ets_printf("STUB: Failed to erase flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  rc = esp_rom_spiflash_write(flash_addr, (uint32_t *)buf, sizeof(buf));
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    ets_printf("STUB: Failed to write flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  rc = esp_rom_spiflash_read(flash_addr, (uint32_t *)buf, sizeof(buf));
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    ets_printf("STUB: Failed to read flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  ets_printf("STUB: ");
  for (int i = 0; i < 10; i++) {
    ets_printf("%x ", buf[i]);
  }
  ets_printf("\n");

  return ret;
}

static int stub_flash_read(uint32_t addr, uint8_t *data, uint32_t size)
{
  int ret = STUB_ERR_OK;
  uint32_t rd_sz = size, flash_addr = addr, read = 0;
  esp_rom_spiflash_result_t rc;
  uint8_t dummy_buf[4];

  if (flash_addr & 0x3UL) {
    rc = esp_rom_spiflash_read(flash_addr & ~0x3UL, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      ets_printf("STUB: Failed to read flash @ 0x%x (%d)\n", flash_addr & ~0x3UL, rc);
      return STUB_ERR_FAIL;
    }
    uint32_t sz = 4 - (flash_addr & 0x3UL);
    ets_printf("STUB: Read flash dword @ 0x%x sz %d\n", flash_addr & ~0x3UL, sz);
    memcpy(data, &dummy_buf[flash_addr & 0x3UL], sz);
    rd_sz -= sz;
    read += sz;
    flash_addr = (flash_addr + 0x3UL) & ~0x3UL;
  }

  if (rd_sz & 0x3UL) {
    rd_sz = rd_sz & ~0x3UL;
  }
  ets_printf("STUB: Read flash @ 0x%x sz %d\n", flash_addr, rd_sz);
  rc = esp_rom_spiflash_read(flash_addr, (uint32_t *)&data[read], rd_sz);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    ets_printf("STUB: Failed to read flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }
  read += rd_sz;
  if (read < size) {
    rc = esp_rom_spiflash_read(flash_addr + rd_sz, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      ets_printf("STUB: Failed to read flash @ 0x%x (%d)\n", flash_addr + rd_sz, rc);
      return STUB_ERR_FAIL;
    }
    ets_printf("STUB: Read flash dword @ 0x%x sz %d\n", flash_addr + rd_sz, size - read);
    memcpy(&data[read], dummy_buf, size - read);
  }
  //TODO: remove debug print
  ets_printf("STUB: ");
  for (int i = 0; i < size; i++) {
    ets_printf("%x ", data[i]);
  }
  ets_printf("\n");

  return ret;
}

static int stub_flash_write(uint32_t addr, uint8_t *data, uint32_t size)
{
  int ret = STUB_ERR_OK;
  esp_rom_spiflash_result_t rc;
  uint8_t dummy_buf[4];
  uint32_t wr_sz = size, flash_addr = addr, written = 0;

  if (flash_addr & 0x3UL) {
    rc = esp_rom_spiflash_read(flash_addr & ~0x3UL, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      ets_printf("STUB: Failed to read flash @ 0x%x (%d)\n", flash_addr & ~0x3UL, rc);
      return STUB_ERR_FAIL;
    }
    uint32_t sz = 4 - (flash_addr & 0x3UL);
    ets_printf("STUB: Write flash dword @ 0x%x sz %d\n", flash_addr & ~0x3UL, sz);
    memcpy(&dummy_buf[flash_addr & 0x3UL], data, sz);
    rc = esp_rom_spiflash_write(flash_addr & ~0x3UL, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      ets_printf("STUB: Failed to write flash (%d)\n", rc);
      return STUB_ERR_FAIL;
    }
    wr_sz -= sz;
    written += sz;
    flash_addr = (flash_addr + 0x3UL) & ~0x3UL;
  }

  if (wr_sz & 0x3UL) {
    wr_sz = wr_sz & ~0x3UL;
  }
  ets_printf("STUB: Write flash @ 0x%x sz %d\n", flash_addr, wr_sz);
  rc = esp_rom_spiflash_write(flash_addr, (uint32_t *)&data[written], wr_sz);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    ets_printf("STUB: Failed to write flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  written += wr_sz;
  if (written < size) {
    rc = esp_rom_spiflash_read(flash_addr + wr_sz, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      ets_printf("STUB: Failed to read flash @ 0x%x (%d)\n", flash_addr + wr_sz, rc);
      return STUB_ERR_FAIL;
    }
    ets_printf("STUB: Write flash dword @ 0x%x sz %d\n", flash_addr + wr_sz, size - written);
    memcpy(dummy_buf, &data[written], size - written);
    rc = esp_rom_spiflash_write(flash_addr + wr_sz, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      ets_printf("STUB: Failed to write flash (%d)\n", rc);
      return STUB_ERR_FAIL;
    }
  }
  return ret;
}

static int stub_flash_erase(uint32_t flash_addr, uint32_t size)
{
  int ret = STUB_ERR_OK;

  if (flash_addr & (SPI_FLASH_SEC_SIZE-1)) {
    flash_addr &= ~(SPI_FLASH_SEC_SIZE-1);
  }

  if (size & (SPI_FLASH_SEC_SIZE-1)) {
    size = (size + (SPI_FLASH_SEC_SIZE-1)) & ~(SPI_FLASH_SEC_SIZE-1);
  }

  ets_printf("STUB: erase flash @ 0x%x, sz %d \n", flash_addr, size);
  esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_area(flash_addr, size);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    ets_printf("STUB: Failed to erase flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  return ret;
}

static int stub_flash_handler(int cmd, va_list ap)
{
  int ret;
  uint32_t core_id = stub_get_coreid();
  uint32_t other_core_id = core_id == 0 ? 1 : 0;
  uint32_t flags[2];
  uint32_t flash_addr = va_arg(ap, uint32_t);
  uint32_t size = va_arg(ap, uint32_t);
  uint8_t *buf = va_arg(ap, uint8_t *);

  ets_printf("STUB: flash a %x, b %x, s %d\n", flash_addr, buf, size);

  stub_spi_flash_disable_cache(other_core_id, &flags[1]);
  stub_spi_flash_disable_cache(core_id, &flags[0]);

  esp_rom_spiflash_result_t rc = esp_rom_spiflash_unlock();
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    ets_printf("STUB: Failed to unlock flash (%d)\n", rc);
    ret = STUB_ERR_FAIL;
    goto _flash_end;
  }

  switch (cmd) {
    case STUB_CMD_FLASH_READ:
      ret = stub_flash_read(flash_addr, buf, size);
      break;
    case STUB_CMD_FLASH_ERASE:
      ret = stub_flash_erase(flash_addr, size);
      break;
    case STUB_CMD_FLASH_WRITE:
      ret = stub_flash_write(flash_addr, buf, size);
      break;
    case STUB_CMD_FLASH_TEST:
      ret = stub_flash_test();
      break;
    default:
      ret = STUB_ERR_NOT_SUPPORTED;
  }

_flash_end:
  stub_spi_flash_restore_cache(core_id, flags[0]);
  stub_spi_flash_restore_cache(other_core_id, flags[1]);

  return ret;
}

int stub_main(int cmd, ...)
{
  va_list ap;
  int ret = 0;

  //TODO: temporarily relocate vector

  /* this points to stub_main now, clear for next boot */
  //FIXME: do we need this for JTAG flasher?
  ets_set_user_start(0);

  /* zero bss */
  for(uint32_t *p = &_bss_start; p < &_bss_end; p++) {
    *p = 0;
  }

  //TODO: disable interrupts globally
  //TODO: UART initialization
  ets_printf("STUB: main %d\n", cmd);
  //g_stub_stack[0] = 1;

  va_start(ap, cmd);

  switch (cmd) {
    case STUB_CMD_TEST:
      ret = stub_test(cmd);
      break;
    case STUB_CMD_FLASH_READ:
    case STUB_CMD_FLASH_ERASE:
    case STUB_CMD_FLASH_WRITE:
    case STUB_CMD_FLASH_TEST:
      ret = stub_flash_handler(cmd, ap);
      break;
    default:
      ret = STUB_ERR_NOT_SUPPORTED;
  }

  va_end(ap);

  return ret;
}
