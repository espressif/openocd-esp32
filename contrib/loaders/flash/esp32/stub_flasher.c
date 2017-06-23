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
#include "rom/uart.h"
#include "xtensa/hal.h"
#include "eri.h"
#include "trax.h"
#include "esp_app_trace.h"

#define STUB_ASYNC_WRITE_ALGO   0
#define STUB_USE_APPTRACE       1

#define SPI_FLASH_SEC_SIZE      4096    /**< SPI Flash sector size */

#define ESP_APPTRACE_TRAX_BLOCK_SIZE            (0x4000UL)
#define ESP_APPTRACE_USR_DATA_LEN_MAX           (ESP_APPTRACE_TRAX_BLOCK_SIZE - 2)

#define STUB_ERR_OK             0
#define STUB_ERR_FAIL           (-1)
#define STUB_ERR_NOT_SUPPORTED  (-2)

#define STUB_CMD_TEST           0
#define STUB_CMD_FLASH_READ     1
#define STUB_CMD_FLASH_WRITE    2
#define STUB_CMD_FLASH_ERASE    3
#define STUB_CMD_FLASH_TEST     4

#define ESP_APPTRACE_TRAX_CTRL_REG              ERI_TRAX_DELAYCNT
#define ESP_APPTRACE_TRAX_HOST_CONNECT          (1 << 23)

#define STUB_LOG_NONE           0
#define STUB_LOG_ERROR          1
#define STUB_LOG_WARN           2
#define STUB_LOG_INFO           3
#define STUB_LOG_DEBUG          4
#define STUB_LOG_VERBOSE        5

#define STUB_LOG_LOCAL_LEVEL  STUB_LOG_VERBOSE

#define STUB_LOG( level, format, ... )   \
    do { \
        if (STUB_LOG_LOCAL_LEVEL >= level) { \
            ets_printf(format, ##__VA_ARGS__); \
        } \
    } while(0)

#define STUB_LOGE( format, ... )  STUB_LOG(STUB_LOG_ERROR, "STUB_E: " format, ##__VA_ARGS__)
#define STUB_LOGW( format, ... )  STUB_LOG(STUB_LOG_WARN, "STUB_W: "format, ##__VA_ARGS__)
#define STUB_LOGI( format, ... )  STUB_LOG(STUB_LOG_INFO, "STUB_I: "format, ##__VA_ARGS__)
#define STUB_LOGD( format, ... )  STUB_LOG(STUB_LOG_DEBUG, "STUB_D: "format, ##__VA_ARGS__)
#define STUB_LOGV( format, ... )  STUB_LOG(STUB_LOG_VERBOSE, "STUB_V: "format, ##__VA_ARGS__)
#define STUB_LOGO( format, ... )  STUB_LOG(STUB_LOG_NONE, format, ##__VA_ARGS__)

//#define XT_CLOCK_FREQ 240000000UL // TODO: get clock from PLL config
#define CPUTICKS2US(_t_)       ((_t_)/(XT_CLOCK_FREQ/1000000))

extern uint32_t _bss_start;
extern uint32_t _bss_end;

#if STUB_USE_APPTRACE
/* used in app trace module */
uint32_t esp_log_early_timestamp()
{
  return 0;//xthal_get_ccount();
}
#endif

void __assert_func(const char *path, int line, const char *func, const char *msg)// _ATTRIBUTE ((__noreturn__))
{
    STUB_LOGO("ASSERT at %s:%d '%s'\n", func, line, msg);
    while (1);
}

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

static int stub_flash_test(void)
{
  int ret = STUB_ERR_OK;
  uint8_t buf[32] = {9, 1, 2, 3, 4, 5, 6, 8};
  uint32_t flash_addr = 0x1d4000;

  esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_sector(flash_addr/SPI_FLASH_SEC_SIZE);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    STUB_LOGE("Failed to erase flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  rc = esp_rom_spiflash_write(flash_addr, (uint32_t *)buf, sizeof(buf));
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    STUB_LOGE("Failed to write flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  rc = esp_rom_spiflash_read(flash_addr, (uint32_t *)buf, sizeof(buf));
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    STUB_LOGE("Failed to read flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  STUB_LOGD("Data: ");
  for (int i = 0; i < 10; i++) {
    STUB_LOGO("%x ", buf[i]);
  }
  STUB_LOGO("\n");

  return ret;
}

#if STUB_USE_APPTRACE
static int stub_flash_read_loop(uint32_t addr, uint32_t size)
{
    esp_rom_spiflash_result_t rc;
    uint32_t total_cnt = 0;
    STUB_LOGD("Start reading %d bytes @ 0x%x\n", size, addr);

    while (total_cnt < size) {
      uint32_t rd_sz = size - total_cnt > ESP_APPTRACE_USR_DATA_LEN_MAX ? ESP_APPTRACE_USR_DATA_LEN_MAX : size - total_cnt;
      if (rd_sz & 0x3UL) {
        rd_sz &= ~0x3UL;
      }
      if (rd_sz == 0) {
        break;
      }
      uint32_t start = xthal_get_ccount();
      uint8_t *buf = esp_apptrace_buffer_get(ESP_APPTRACE_DEST_TRAX, rd_sz, ESP_APPTRACE_TMO_INFINITE);
      if (!buf) {
        STUB_LOGE("Failed to get trace buf!\n");
        return STUB_ERR_FAIL;
      }
      uint32_t end = xthal_get_ccount();
      STUB_LOGD("Got trace buf %d bytes @ 0x%x in %d ms\n", rd_sz, buf, CPUTICKS2US(end - start)/1000);

      start = xthal_get_ccount();
      rc = esp_rom_spiflash_read(addr + total_cnt, (uint32_t *)buf, rd_sz);
      end = xthal_get_ccount();
      STUB_LOGD("Read flash @ 0x%x sz %d in %d ms\n", addr + total_cnt, rd_sz, CPUTICKS2US(end - start)/1000);
      if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
        STUB_LOGE("Failed to read flash (%d)!\n", rc);
        esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX, buf, ESP_APPTRACE_TMO_INFINITE);
        return STUB_ERR_FAIL;
      }
      total_cnt += rd_sz;

      esp_err_t err = esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX, buf, ESP_APPTRACE_TMO_INFINITE);
      if (err != ESP_OK) {
        STUB_LOGE("Failed to put trace buf!\n");
        return STUB_ERR_FAIL;
      }
      err = esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
      if (err != ESP_OK) {
        STUB_LOGE("Failed to flush trace buf!\n");
        return STUB_ERR_FAIL;
      }
      STUB_LOGE("Sent trace buf %d bytes @ 0x%x\n", rd_sz, buf);
    }

    if (total_cnt < size) {
      if ((size - total_cnt) >= 4) {
        STUB_LOGE("Exited loop when remaing data size is more the 4 bytes!\n");
        return STUB_ERR_FAIL; /*should never get here*/
      }
      // if we exited loop because remaing data size is less than 4 bytes
      uint8_t last_bytes[4];
      rc = esp_rom_spiflash_read(addr + total_cnt, (uint32_t *)last_bytes, 4);
      STUB_LOGD("Read padded word from flash @ 0x%x\n", addr + total_cnt);
      if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
        STUB_LOGE("Failed to read last word from flash (%d)!\n", rc);
        return STUB_ERR_FAIL;
      }
      uint8_t *buf = esp_apptrace_buffer_get(ESP_APPTRACE_DEST_TRAX, size - total_cnt, ESP_APPTRACE_TMO_INFINITE);
      if (!buf) {
        STUB_LOGE("Failed to get trace buf!\n");
        return STUB_ERR_FAIL;
      }
      memcpy(buf, last_bytes, size - total_cnt);
      esp_err_t err = esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX, buf, ESP_APPTRACE_TMO_INFINITE);
      if (err != ESP_OK) {
        STUB_LOGE("Failed to put trace buf!\n");
        return STUB_ERR_FAIL;
      }
      err = esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
      if (err != ESP_OK) {
        STUB_LOGE("Failed to flush trace buf!\n");
        return STUB_ERR_FAIL;
      }
      STUB_LOGE("Sent last trace buf %d bytes @ 0x%x\n", size - total_cnt, buf);
    }

    STUB_LOGD("Read %d bytes @ 0x%x\n", size, addr);

    return STUB_ERR_OK;
}
#else
static int stub_flash_read(uint32_t addr, uint8_t *data, uint32_t size)
{
  int ret = STUB_ERR_OK;
  uint32_t rd_sz = size, flash_addr = addr, read = 0;
  esp_rom_spiflash_result_t rc;
  uint8_t dummy_buf[4];

  if (flash_addr & 0x3UL) {
    rc = esp_rom_spiflash_read(flash_addr & ~0x3UL, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      STUB_LOGE("Failed to read flash @ 0x%x (%d)\n", flash_addr & ~0x3UL, rc);
      return STUB_ERR_FAIL;
    }
    uint32_t sz = 4 - (flash_addr & 0x3UL);
    STUB_LOGD("Read flash dword @ 0x%x sz %d\n", flash_addr & ~0x3UL, sz);
    memcpy(data, &dummy_buf[flash_addr & 0x3UL], sz);
    rd_sz -= sz;
    read += sz;
    flash_addr = (flash_addr + 0x3UL) & ~0x3UL;
  }

  if (rd_sz & 0x3UL) {
    rd_sz = rd_sz & ~0x3UL;
  }
  uint32_t start = xthal_get_ccount();
  rc = esp_rom_spiflash_read(flash_addr, (uint32_t *)&data[read], rd_sz);
  uint32_t end = xthal_get_ccount();
  STUB_LOGD("Read flash @ 0x%x sz %d in %d ms\n", flash_addr, rd_sz, CPUTICKS2US(end - start)/1000);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    STUB_LOGE("Failed to read flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }
  read += rd_sz;
  if (read < size) {
    rc = esp_rom_spiflash_read(flash_addr + rd_sz, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      STUB_LOGE("Failed to read flash @ 0x%x (%d)\n", flash_addr + rd_sz, rc);
      return STUB_ERR_FAIL;
    }
    STUB_LOGD("Read flash dword @ 0x%x sz %d\n", flash_addr + rd_sz, size - read);
    memcpy(&data[read], dummy_buf, size - read);
  }
  //TODO: remove debug print
  STUB_LOGD("DATA: ");
  for (int i = 0; i < 32; i++) {
    STUB_LOGO("%x ", data[i]);
  }
  STUB_LOGO("\n");

  return ret;
}
#endif

#if STUB_USE_APPTRACE
static int stub_flash_write_loop(uint32_t addr, uint32_t size)
{
    esp_rom_spiflash_result_t rc;
    uint32_t total_cnt = 0;
    uint8_t cached_bytes_num = 0, cached_bytes[4];
    STUB_LOGD("Start writing %d bytes @ 0x%x\n", size, addr);

    //TODO: check alignment
    while (total_cnt < size) {
      uint32_t wr_sz = size - total_cnt - cached_bytes_num;
      STUB_LOGD("Req trace down buf %d bytes %d-%d-%d\n", wr_sz, size, total_cnt, cached_bytes_num);
      uint32_t start = xthal_get_ccount();
      uint8_t *wr_p, *buf = esp_apptrace_down_buffer_get(ESP_APPTRACE_DEST_TRAX, &wr_sz, ESP_APPTRACE_TMO_INFINITE);
      if (!buf) {
        STUB_LOGE("Failed to get trace down buf!\n");
        return STUB_ERR_FAIL;
      }
      uint32_t end = xthal_get_ccount();
      STUB_LOGD("Got trace down buf %d bytes @ 0x%x in %d ms\n", wr_sz, buf, CPUTICKS2US(end - start)/1000);

      wr_p = buf;
      if (cached_bytes_num != 0) {
        // add cached bytes from the end of the prev buffer to the starting bytes of the current one
        memcpy(&cached_bytes[cached_bytes_num], buf, 4-cached_bytes_num);
        rc = esp_rom_spiflash_write(addr + total_cnt, (uint32_t *)cached_bytes, 4);
        STUB_LOGD("Write padded word to flash @ 0x%x\n", addr + total_cnt);
        if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
          STUB_LOGE("Failed to write flash (%d)\n", rc);
          esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX, buf, ESP_APPTRACE_TMO_INFINITE);
          return STUB_ERR_FAIL;
        }
        wr_p += 4-cached_bytes_num;
        wr_sz -= 4-cached_bytes_num;
        total_cnt += 4;
        cached_bytes_num = 0;
      }
      if (wr_sz & 0x3UL) {
        cached_bytes_num = wr_sz & 0x3UL;
        wr_sz &= ~0x3UL;
        memcpy(cached_bytes, wr_p + wr_sz, cached_bytes_num);
      }
      // write buffer with aligned size
      if (wr_sz) {
        start = xthal_get_ccount();
        rc = esp_rom_spiflash_write(addr + total_cnt, (uint32_t *)wr_p, wr_sz);
        end = xthal_get_ccount();
        STUB_LOGD("Write flash @ 0x%x sz %d in %d ms\n", addr + total_cnt, wr_sz, CPUTICKS2US(end - start)/1000);
        if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
          STUB_LOGE("Failed to write flash (%d)\n", rc);
          esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX, buf, ESP_APPTRACE_TMO_INFINITE);
          return STUB_ERR_FAIL;
        }
        total_cnt += wr_sz;
      }
      // free buffer
      esp_err_t err = esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX, buf, ESP_APPTRACE_TMO_INFINITE);
      if (err != ESP_OK) {
        STUB_LOGE("Failed to put trace buf!\n");
        return STUB_ERR_FAIL;
      }
      // STUB_LOGE("Recvd trace down buf %d bytes @ 0x%x\n", wr_sz, buf);
    }

    if (cached_bytes_num != 0) {
      // add padding to cached bytes from the end of the last buffer
      while (cached_bytes_num & 0x3UL) {
        cached_bytes[cached_bytes_num++] = 0xFF;
      }
      rc = esp_rom_spiflash_write(addr + total_cnt, (uint32_t *)cached_bytes, 4);
      STUB_LOGD("Write last padded word to flash @ 0x%x\n", addr + total_cnt);
      if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
        STUB_LOGE("Failed to write flash (%d)\n", rc);
        return STUB_ERR_FAIL;
      }
    }

    STUB_LOGD("Wrote %d bytes @ 0x%x\n", size, addr);

    return STUB_ERR_OK;
}
#else
static int stub_flash_write(uint32_t addr, uint8_t *data, uint32_t size)
{
  int ret = STUB_ERR_OK;
  esp_rom_spiflash_result_t rc;
  uint8_t dummy_buf[4];
  uint32_t wr_sz = size, flash_addr = addr, written = 0;

  if (flash_addr & 0x3UL) {
    rc = esp_rom_spiflash_read(flash_addr & ~0x3UL, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      STUB_LOGE("Failed to read flash @ 0x%x (%d)\n", flash_addr & ~0x3UL, rc);
      return STUB_ERR_FAIL;
    }
    uint32_t sz = 4 - (flash_addr & 0x3UL);
    STUB_LOGD("Write flash dword @ 0x%x sz %d\n", flash_addr & ~0x3UL, sz);
    memcpy(&dummy_buf[flash_addr & 0x3UL], data, sz);
    rc = esp_rom_spiflash_write(flash_addr & ~0x3UL, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      STUB_LOGE("Failed to write flash (%d)\n", rc);
      return STUB_ERR_FAIL;
    }
    wr_sz -= sz;
    written += sz;
    flash_addr = (flash_addr + 0x3UL) & ~0x3UL;
  }

  if (wr_sz & 0x3UL) {
    wr_sz = wr_sz & ~0x3UL;
  }
  uint32_t start = xthal_get_ccount();
  rc = esp_rom_spiflash_write(flash_addr, (uint32_t *)&data[written], wr_sz);
  uint32_t end = xthal_get_ccount();
  STUB_LOGD("Write flash @ 0x%x sz %d in %d ms\n", flash_addr, wr_sz, CPUTICKS2US(end - start)/1000);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    STUB_LOGE("Failed to write flash (%d)\n", rc);
    return STUB_ERR_FAIL;
  }

  written += wr_sz;
  if (written < size) {
    rc = esp_rom_spiflash_read(flash_addr + wr_sz, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      STUB_LOGE("Failed to read flash @ 0x%x (%d)\n", flash_addr + wr_sz, rc);
      return STUB_ERR_FAIL;
    }
    STUB_LOGD("Write flash dword @ 0x%x sz %d\n", flash_addr + wr_sz, size - written);
    memcpy(dummy_buf, &data[written], size - written);
    rc = esp_rom_spiflash_write(flash_addr + wr_sz, (uint32_t *)dummy_buf, sizeof(dummy_buf));
    if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
      STUB_LOGE("Failed to write flash (%d)\n", rc);
      return STUB_ERR_FAIL;
    }
  }
  return ret;
}
#endif

static int stub_flash_erase(uint32_t flash_addr, uint32_t size)
{
  int ret = STUB_ERR_OK;

  if (flash_addr & (SPI_FLASH_SEC_SIZE-1)) {
    flash_addr &= ~(SPI_FLASH_SEC_SIZE-1);
  }

  if (size & (SPI_FLASH_SEC_SIZE-1)) {
    size = (size + (SPI_FLASH_SEC_SIZE-1)) & ~(SPI_FLASH_SEC_SIZE-1);
  }

  STUB_LOGD("erase flash @ 0x%x, sz %d \n", flash_addr, size);
  esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_area(flash_addr, size);
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    STUB_LOGE("Failed to erase flash (%d)\n", rc);
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
  uint8_t *down_buf = va_arg(ap, uint8_t *);
  uint32_t down_size = va_arg(ap, uint32_t);

  STUB_LOGD("flash a %x, s %d\n", flash_addr, size);

  STUB_LOGI("Init apptrace module db 0x%x, sz %d\n", down_buf, down_size);
  esp_err_t err = esp_apptrace_init();
  if (err != ESP_OK) {
    STUB_LOGE("Failed to init apptrace module (%d)!\n", err);
    return STUB_ERR_FAIL;
  }
  // imply that host is auto-connected
  uint32_t reg = eri_read(ESP_APPTRACE_TRAX_CTRL_REG);
  reg |= ESP_APPTRACE_TRAX_HOST_CONNECT;
  eri_write(ESP_APPTRACE_TRAX_CTRL_REG, reg);

  if (down_size) {
    esp_apptrace_down_buffer_config(down_buf, down_size);
  }

  stub_spi_flash_disable_cache(other_core_id, &flags[1]);
  stub_spi_flash_disable_cache(core_id, &flags[0]);

  esp_rom_spiflash_result_t rc = esp_rom_spiflash_unlock();
  if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
    STUB_LOGE("Failed to unlock flash (%d)\n", rc);
    ret = STUB_ERR_FAIL;
    goto _flash_end;
  }

  switch (cmd) {
    case STUB_CMD_FLASH_READ:
#if STUB_USE_APPTRACE
      ret = stub_flash_read_loop(flash_addr, size);
#else
      ret = stub_flash_read(flash_addr, buf, size);
#endif
      break;
    case STUB_CMD_FLASH_ERASE:
      ret = stub_flash_erase(flash_addr, size);
      break;
    case STUB_CMD_FLASH_WRITE:
#if STUB_USE_APPTRACE
      ret = stub_flash_write_loop(flash_addr, size);
#else
      ret = stub_flash_write(flash_addr, buf, size);
#endif
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

  /* zero bss */
  for(uint32_t *p = &_bss_start; p < &_bss_end; p++) {
    *p = 0;
  }

  // we get here just after OpenOCD jumper stub
  // up to 5 parameters are passed via registers by that jumping code
  // interrupts level in PS is set to one to allow high prio IRQs only (including Debug Interrupt)
  // We need Debug Interrupt Level to allow breakpoints handling by OpenOCD

  //TODO: temporarily relocate vector
  //TODO: reset all SW memory mappings (it is better to do in OpenOCD)

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
  uartAttach();
  ets_install_uart_printf();
#endif

  STUB_LOGD("BSS 0x%x..0x%x\n", &_bss_start, &_bss_end);
  STUB_LOGD("cmd %d\n", cmd);

  // for (;;){//uint32_t i = 0; ; i++) {
  //   uint32_t reg = eri_read(ESP_APPTRACE_TRAX_CTRL_REG);
  //   //reg |= ESP_APPTRACE_TRAX_HOST_CONNECT;
  //   //eri_write(ESP_APPTRACE_TRAX_CTRL_REG, i & 0x7FFFUL);
  //   STUB_LOGI("TRAX_CTRL 0x%x\n", reg);
  // }
  // return ret;

  va_start(ap, cmd);

  switch (cmd) {
    case STUB_CMD_TEST:
      STUB_LOGD("TEST %d\n", cmd);
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

  STUB_LOGD("exit %d\n", ret);
  return ret;
}
