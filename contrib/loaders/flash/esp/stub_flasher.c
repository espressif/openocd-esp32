/***************************************************************************
 *   ESP flasher stub                                                      *
 *   Copyright (C) 2017-2019 Espressif Systems Ltd.                        *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#include <stdarg.h>
#include <string.h>
#include "rom/spi_flash.h"
#include "eri.h"
#include "trax.h"
#include "esp_app_trace.h"
#include "esp_flash_partitions.h"
#include "esp_image_format.h"
#include "stub_flasher.h"
#include "stub_flasher_int.h"
#include "stub_flasher_chip.h"
#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
#include "soc/rtc.h"
#include "rom/uart.h"
#endif

#define STUB_DEBUG    0

#define STUB_BP_INSN_SECT_BUF_SIZE        (2*STUB_FLASH_SECTOR_SIZE)

#define ESP_APPTRACE_TRAX_BLOCK_SIZE    (0x4000UL)
#define ESP_APPTRACE_USR_DATA_LEN_MAX   (ESP_APPTRACE_TRAX_BLOCK_SIZE - 2)

#define ESP_APPTRACE_TRAX_CTRL_REG      ERI_TRAX_DELAYCNT
#define ESP_APPTRACE_TRAX_HOST_CONNECT  (1 << 23)

#define XT_INS_BREAK    0x004000
#define XT_INS_BREAKN   0xF02D

#ifdef XT_CLOCK_FREQ
#undef XT_CLOCK_FREQ
#endif
#define XT_CLOCK_FREQ         (esp_clk_cpu_freq())
#define CPUTICKS2US(_t_)      ((_t_)/(XT_CLOCK_FREQ/1000000))

extern uint32_t _bss_start;
extern uint32_t _bss_end;

extern uint32_t stub_flash_get_id(void);
extern void stub_flash_cache_flush(void);
extern void stub_flash_state_prepare(struct stub_flash_state *state);
extern void stub_flash_state_restore(struct stub_flash_state *state);
extern void stub_clock_configure(void);
extern uint32_t esp_clk_cpu_freq(void);

/* used in app trace module */
uint32_t esp_log_early_timestamp()
{
	return 0;
}

void __assert_func(const char *path, int line, const char *func, const char *msg)
{
	STUB_LOGE("ASSERT at %s:%d '%s'\n", func, line, msg);
	while (1) ;
}

static inline uint8_t xtensa_get_insn_size(uint8_t *insn)
{
	return insn[0] & 0x8 ? 2 : 3;
}

#if STUB_DEBUG
static int stub_flash_test(void)
{
	int ret = ESP_XTENSA_STUB_ERR_OK;
	uint8_t buf[32] = {9, 1, 2, 3, 4, 5, 6, 8};
	uint32_t flash_addr = 0x1d4000;

	esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_sector(
		flash_addr / STUB_FLASH_SECTOR_SIZE);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to erase flash (%d)\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}

	rc = esp_rom_spiflash_write(flash_addr, (uint32_t *)buf, sizeof(buf));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to write flash (%d)\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}

	rc = esp_rom_spiflash_read(flash_addr, (uint32_t *)buf, sizeof(buf));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read flash (%d)\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}

	STUB_LOGD("Data: ");
	for (int i = 0; i < 10; i++)
		STUB_LOGD("%x ", buf[i]);
	STUB_LOGD("\n");

	return ret;
}
#endif

static int stub_apptrace_init()
{
	STUB_LOGI("Init apptrace module\n");
	esp_err_t err = esp_apptrace_init();
	if (err != ESP_OK) {
		STUB_LOGE("Failed to init apptrace module (%d)!\n", err);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}
	/* imply that host is auto-connected */
	uint32_t reg = eri_read(ESP_APPTRACE_TRAX_CTRL_REG);
	reg |= ESP_APPTRACE_TRAX_HOST_CONNECT;
	eri_write(ESP_APPTRACE_TRAX_CTRL_REG, reg);

	return ESP_XTENSA_STUB_ERR_OK;
}

static int stub_flash_read(uint32_t addr, uint32_t size)
{
	esp_rom_spiflash_result_t rc;
	uint32_t total_cnt = 0;

	int ret = stub_apptrace_init();
	if (ret != ESP_XTENSA_STUB_ERR_OK)
		return ret;

	STUB_LOGI("Start reading %d bytes @ 0x%x\n", size, addr);

	while (total_cnt < size) {
		uint32_t rd_sz = size - total_cnt >
			ESP_APPTRACE_USR_DATA_LEN_MAX ? ESP_APPTRACE_USR_DATA_LEN_MAX : size -
			total_cnt;
		if (rd_sz & 0x3UL)
			rd_sz &= ~0x3UL;
		if (rd_sz == 0)
			break;
		uint32_t start = xthal_get_ccount();
		uint8_t *buf = esp_apptrace_buffer_get(ESP_APPTRACE_DEST_TRAX,
			rd_sz,
			ESP_APPTRACE_TMO_INFINITE);
		if (!buf) {
			STUB_LOGE("Failed to get trace buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		uint32_t end = xthal_get_ccount();
		STUB_LOGD("Got trace buf %d bytes @ 0x%x in %d ms\n", rd_sz, buf,
			CPUTICKS2US(end - start) / 1000);

		start = xthal_get_ccount();
		rc = esp_rom_spiflash_read(addr + total_cnt, (uint32_t *)buf, rd_sz);
		end = xthal_get_ccount();
		STUB_LOGD("Read flash @ 0x%x sz %d in %d ms\n",
			addr + total_cnt,
			rd_sz,
			CPUTICKS2US(end - start) / 1000);
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read flash (%d)!\n", rc);
			esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX,
				buf,
				ESP_APPTRACE_TMO_INFINITE);
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		total_cnt += rd_sz;

		esp_err_t err = esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX,
			buf,
			ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to put trace buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		STUB_LOGD("Flush trace buf %d bytes @ 0x%x [%x %x %x %x %x %x %x %x]\n",
			rd_sz, buf, buf[-4], buf[-3], buf[-2], buf[-1],
			buf[0], buf[1], buf[2], buf[3]);
		err = esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to flush trace buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		STUB_LOGD("Sent trace buf %d bytes @ 0x%x\n", rd_sz, buf);
	}

	if (total_cnt < size) {
		if ((size - total_cnt) >= 4) {
			STUB_LOGE("Exited loop when remaing data size is more the 4 bytes!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;/*should never get here*/
		}
		/* if we exited loop because remaing data size is less than 4 bytes */
		uint8_t last_bytes[4];
		rc = esp_rom_spiflash_read(addr + total_cnt, (uint32_t *)last_bytes, 4);
		STUB_LOGD("Read padded word from flash @ 0x%x\n", addr + total_cnt);
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read last word from flash (%d)!\n", rc);
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		uint8_t *buf = esp_apptrace_buffer_get(ESP_APPTRACE_DEST_TRAX,
			size - total_cnt,
			ESP_APPTRACE_TMO_INFINITE);
		if (!buf) {
			STUB_LOGE("Failed to get trace buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		memcpy(buf, last_bytes, size - total_cnt);
		esp_err_t err = esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX,
			buf,
			ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to put trace buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		STUB_LOGD("Flush trace buf %d bytes @ 0x%x [%x %x %x %x %x %x %x %x]\n",
			size - total_cnt, buf, buf[-4], buf[-3], buf[-2], buf[-1],
			buf[0], buf[1], buf[2], buf[3]);
		err = esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to flush trace buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		STUB_LOGE("Sent last trace buf %d bytes @ 0x%x\n", size - total_cnt, buf);
	}
	STUB_LOGD("Read %d bytes @ 0x%x\n", size, addr);

	return ESP_XTENSA_STUB_ERR_OK;
}

static int stub_flash_write(uint32_t addr, uint32_t size, uint8_t *down_buf, uint32_t down_size)
{
	esp_rom_spiflash_result_t rc;
	uint32_t total_cnt = 0;
	uint8_t cached_bytes_num = 0, cached_bytes[4];
	STUB_LOGD("Start writing %d bytes @ 0x%x\n", size, addr);

	int ret = stub_apptrace_init();
	if (ret != ESP_XTENSA_STUB_ERR_OK)
		return ret;
	STUB_LOGI("Init apptrace module down buffer %d bytes @ 0x%x\n", down_size, down_buf);
	esp_apptrace_down_buffer_config(down_buf, down_size);

	while (total_cnt < size) {
		uint32_t wr_sz = size - total_cnt - cached_bytes_num;
		STUB_LOGD("Req trace down buf %d bytes %d-%d-%d\n",
			wr_sz,
			size,
			total_cnt,
			cached_bytes_num);
		uint32_t start = xthal_get_ccount();
		uint8_t *wr_p, *buf = esp_apptrace_down_buffer_get(ESP_APPTRACE_DEST_TRAX,
			&wr_sz,
			ESP_APPTRACE_TMO_INFINITE);
		if (!buf) {
			STUB_LOGE("Failed to get trace down buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		uint32_t end = xthal_get_ccount();
		STUB_LOGD("Got trace down buf %d bytes @ 0x%x in %d ms\n", wr_sz, buf,
			CPUTICKS2US(end - start) / 1000);

		wr_p = buf;
		if (cached_bytes_num != 0) {
			/* add cached bytes from the end of the prev buffer to the starting bytes of
			 * the current one */
			memcpy(&cached_bytes[cached_bytes_num], buf, 4 - cached_bytes_num);
			rc = esp_rom_spiflash_write(addr + total_cnt, (uint32_t *)cached_bytes, 4);
			STUB_LOGD("Write padded word [%x %x %x %x] to flash @ 0x%x\n",
				cached_bytes[0],
				cached_bytes[1],
				cached_bytes[2],
				cached_bytes[3],
				addr + total_cnt);
			if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
				STUB_LOGE("Failed to write flash (%d)\n", rc);
				esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX,
					buf,
					ESP_APPTRACE_TMO_INFINITE);
				return ESP_XTENSA_STUB_ERR_FAIL;
			}
			wr_p += 4 - cached_bytes_num;
			wr_sz -= 4 - cached_bytes_num;
			total_cnt += 4;
			cached_bytes_num = 0;
		}
		if (wr_sz & 0x3UL) {
			cached_bytes_num = wr_sz & 0x3UL;
			wr_sz &= ~0x3UL;
			memcpy(cached_bytes, wr_p + wr_sz, cached_bytes_num);
		}
		/* write buffer with aligned size */
		if (wr_sz) {
			start = xthal_get_ccount();
			rc = esp_rom_spiflash_write(addr + total_cnt, (uint32_t *)wr_p, wr_sz);
			end = xthal_get_ccount();
			STUB_LOGD("Write flash @ 0x%x sz %d in %d ms\n",
				addr + total_cnt,
				wr_sz,
				CPUTICKS2US(end - start) / 1000);
			if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
				STUB_LOGE("Failed to write flash (%d)\n", rc);
				esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX,
					buf,
					ESP_APPTRACE_TMO_INFINITE);
				return ESP_XTENSA_STUB_ERR_FAIL;
			}
			total_cnt += wr_sz;
		}
		/* free buffer */
		esp_err_t err = esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX,
			buf,
			ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to put trace buf!\n");
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		STUB_LOGD("Recvd trace down buf %d bytes @ 0x%x\n", wr_sz, buf);
	}

	if (cached_bytes_num != 0) {
		/* add padding to cached bytes from the end of the last buffer */
		while (cached_bytes_num & 0x3UL) {
			cached_bytes[cached_bytes_num++] = 0xFF;
		}
		rc = esp_rom_spiflash_write(addr + total_cnt, (uint32_t *)cached_bytes, 4);
		STUB_LOGD("Write last padded word to flash @ 0x%x\n", addr + total_cnt);
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to write flash (%d)\n", rc);
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
	}

	STUB_LOGD("Wrote %d bytes @ 0x%x\n", size, addr);

	return ESP_XTENSA_STUB_ERR_OK;
}

static int stub_flash_erase(uint32_t flash_addr, uint32_t size)
{
	int ret = ESP_XTENSA_STUB_ERR_OK;

	if (flash_addr & (STUB_FLASH_SECTOR_SIZE - 1))
		flash_addr &= ~(STUB_FLASH_SECTOR_SIZE - 1);

	if (size & (STUB_FLASH_SECTOR_SIZE - 1))
		size = (size + (STUB_FLASH_SECTOR_SIZE - 1)) & ~(STUB_FLASH_SECTOR_SIZE - 1);

	STUB_LOGD("erase flash @ 0x%x, sz %d\n", flash_addr, size);
	esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_area(flash_addr, size);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to erase flash (%d)\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}

	return ret;
}

static int stub_flash_erase_check(uint32_t start_sec, uint32_t sec_num, uint8_t *sec_erased)
{
	int ret = ESP_XTENSA_STUB_ERR_OK;
	uint8_t buf[STUB_FLASH_SECTOR_SIZE / 8];/* implying that sector size is multiple of
						 * sizeof(buf) */

	STUB_LOGD("erase check start %d, sz %d\n", start_sec, sec_num);

	for (int i = start_sec; i < start_sec + sec_num; i++) {
		sec_erased[i] = 1;
		for (int k = 0; k < STUB_FLASH_SECTOR_SIZE / sizeof(buf); k++) {
			esp_rom_spiflash_result_t rc = esp_rom_spiflash_read(
				i * STUB_FLASH_SECTOR_SIZE + k * sizeof(buf),
				(uint32_t *)buf,
				sizeof(buf));
			if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
				STUB_LOGE("Failed to read flash (%d)!\n", rc);
				return ESP_XTENSA_STUB_ERR_FAIL;
			}
			for (int n = 0; n < sizeof(buf); n++) {
				if (buf[n] != 0xFF) {
					sec_erased[i] = 0;
					break;
				}
			}
			if (!sec_erased[i])
				break;
		}
	}

	STUB_LOGD("erase checked\n");

	return ret;
}

static uint32_t stub_flash_get_size(void)
{
	uint32_t size = 0;

	STUB_LOGD("%s: ENTER\n", __func__);

	uint32_t id = stub_flash_get_id();
	switch (id) {
		case 0x12: size = 256 * 1024; break;
		case 0x13: size = 512 * 1024; break;
		case 0x14: size = 1 * 1024 * 1024; break;
		case 0x15: size = 2 * 1024 * 1024; break;
		case 0x16: size = 4 * 1024 * 1024; break;
		case 0x17: size = 8 * 1024 * 1024; break;
		case 0x18: size = 16 * 1024 * 1024; break;
		default:
			size = 0;
	}
	STUB_LOGD("Flash ID %x, size %d KB\n", id, size / 1024);
	return size;
}

static inline bool stub_flash_should_map(uint32_t load_addr)
{
	return (load_addr >= SOC_IROM_LOW && load_addr < SOC_IROM_HIGH)
	       || (load_addr >= SOC_DROM_LOW && load_addr < SOC_DROM_HIGH);
}

static int stub_flash_get_app_mappings(uint32_t off, struct esp_xtensa_flash_mapping *flash_map)
{
	esp_image_header_t img_hdr;
	uint16_t maps_num = 0;

	esp_rom_spiflash_result_t rc =
		esp_rom_spiflash_read(off, (uint32_t *)&img_hdr, sizeof(img_hdr));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read app image header (%d)!\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}
	if (img_hdr.magic != ESP_IMAGE_HEADER_MAGIC) {
		STUB_LOGE("Invalid magic number 0x%x in app image!\n", img_hdr.magic);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}

	STUB_LOGI("Found app image: magic 0x%x, %d segments, entry @ 0x%x\n",
		img_hdr.magic,
		img_hdr.segment_count,
		img_hdr.entry_addr);
	uint32_t flash_addr = off + sizeof(img_hdr);
	for (int k = 0; k < img_hdr.segment_count; k++) {
		esp_image_segment_header_t seg_hdr;
		rc = esp_rom_spiflash_read(flash_addr, (uint32_t *)&seg_hdr, sizeof(seg_hdr));
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read app segment header (%d)!\n", rc);
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		STUB_LOGI("App segment %d: %d bytes @ 0x%x\n",
			k,
			seg_hdr.data_len,
			seg_hdr.load_addr);
		if (stub_flash_should_map(seg_hdr.load_addr)) {
			STUB_LOGI("Mapped segment %d: %d bytes @ 0x%x -> 0x%x\n",
				maps_num,
				seg_hdr.data_len,
				flash_addr + sizeof(seg_hdr),
				seg_hdr.load_addr);
			if (maps_num < ESP_XTENSA_STUB_FLASH_MAPPINGS_MAX_NUM) {
				flash_map->maps[maps_num].phy_addr = flash_addr + sizeof(seg_hdr);
				flash_map->maps[maps_num].load_addr = seg_hdr.load_addr;
				flash_map->maps[maps_num].size = seg_hdr.data_len;
				maps_num++;
			} else
				break;
		}
		flash_addr += sizeof(seg_hdr) + seg_hdr.data_len;
	}
	flash_map->maps_num = maps_num;
	return ESP_XTENSA_STUB_ERR_OK;
}

static int stub_flash_get_map(uint32_t app_off, uint32_t maps_addr)
{
	esp_rom_spiflash_result_t rc;
	esp_partition_info_t part;
	struct esp_xtensa_flash_mapping *flash_map = (struct esp_xtensa_flash_mapping *)maps_addr;
	uint32_t flash_size = stub_flash_get_size();

	STUB_LOGD("%s: 0x%x 0x%x\n", __func__, app_off, maps_addr);
	flash_map->maps_num = 0;
	if (app_off != (uint32_t)-1)
		return stub_flash_get_app_mappings(app_off, flash_map);

	for (uint32_t i = 0;; i++) {
		rc = esp_rom_spiflash_read(
			ESP_PARTITION_TABLE_OFFSET+i*sizeof(esp_partition_info_t),
			(uint32_t *)&part,
			sizeof(part));
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read partitions table entrt (%d)!\n", rc);
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		if (part.magic != ESP_PARTITION_MAGIC) {
			STUB_LOGI("No app partition found 0x%x\n", part.magic);
			break;
		}
		if (part.pos.offset > flash_size || part.pos.offset + part.pos.size > flash_size) {
			STUB_LOGE(
				"Partition %d invalid - offset 0x%x size 0x%x exceeds flash chip size 0x%x\n",
				i,
				part.pos.offset,
				part.pos.size,
				flash_size);
			return ESP_XTENSA_STUB_ERR_FAIL;
		}
		STUB_LOGD("Found partition %d, m 0x%x, t 0x%x, st 0x%x, l '%s'\n",
			i,
			part.magic,
			part.type,
			part.subtype,
			part.label);
		if (part.type == PART_TYPE_APP) {
			STUB_LOGI("Found app partition: '%s' %d KB @ 0x%x\n",
				part.label,
				part.pos.size / 1024,
				part.pos.offset);
			return stub_flash_get_app_mappings(part.pos.offset, flash_map);
		}
	}
	return ESP_XTENSA_STUB_ERR_OK;
}

/**
* Possible BP layouts in flash:
* 1) addr is aligned to 4 bytes (in 1 sector)
* 2) addr is unaligned to 4 bytes, BP is not crossing sector's boundary (in 1 sector)
*   - not crossing 4 bytes alignment boundary
*   - crossing 4 bytes alignment boundary
* 3) addr is unaligned to 4 bytes, BP is crossing sector's boundary (in 2 sectors)
*/
static uint8_t stub_flash_set_bp(uint32_t bp_flash_addr, uint32_t insn_buf_addr, uint8_t *insn_sect)
{
	esp_rom_spiflash_result_t rc;

	STUB_LOGD("%s: 0x%x 0x%x\n", __func__, bp_flash_addr, insn_buf_addr);

	rc = esp_rom_spiflash_read(bp_flash_addr & ~(STUB_FLASH_SECTOR_SIZE - 1),
		(uint32_t *)insn_sect,
		STUB_BP_INSN_SECT_BUF_SIZE);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read insn sector (%d)!\n", rc);
		return 0;
	}
	uint8_t insn_sz =
		xtensa_get_insn_size(&insn_sect[bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)]);
	memcpy((void *)insn_buf_addr,
		&insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1))],
		insn_sz);
	STUB_LOGI("Read insn [%02x %02x %02x] %d bytes @ 0x%x\n",
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 0],
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 1],
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 2],
		insn_sz,
		bp_flash_addr);

	/* this will erase full sector or two */
	if (stub_flash_erase(bp_flash_addr, insn_sz) != ESP_XTENSA_STUB_ERR_OK) {
		STUB_LOGE("Failed to erase insn sector!\n");
		return 0;
	}
	union {
		uint32_t d32;
		uint8_t d8[4];
	} break_insn;
	break_insn.d32 = insn_sz == 2 ? XT_INS_BREAKN : XT_INS_BREAK;
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 0] = break_insn.d8[0];
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 1] = break_insn.d8[1];
	if (insn_sz == 3)
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 2] = break_insn.d8[2];
	rc = esp_rom_spiflash_write(bp_flash_addr & ~(STUB_FLASH_SECTOR_SIZE - 1),
		(uint32_t *)insn_sect,
		STUB_BP_INSN_SECT_BUF_SIZE);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to write break insn (%d)!\n", rc);
		return 0;
	}
	stub_flash_cache_flush();
	return insn_sz;
}

static int stub_flash_clear_bp(uint32_t bp_flash_addr, uint32_t insn_buf_addr, uint8_t *insn_sect)
{
	esp_rom_spiflash_result_t rc;
	uint8_t *insn = (uint8_t *)insn_buf_addr;

	STUB_LOGD("%s: 0x%x 0x%x [%02x %02x %02x]\n",
		__func__,
		bp_flash_addr,
		insn_buf_addr,
		insn[0],
		insn[1],
		insn[2]);

	rc = esp_rom_spiflash_read(bp_flash_addr & ~(STUB_FLASH_SECTOR_SIZE - 1),
		(uint32_t *)insn_sect,
		STUB_BP_INSN_SECT_BUF_SIZE);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read insn sector (%d)!\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}
	uint8_t insn_sz = xtensa_get_insn_size(insn);
	/* this will erase full sector or two */
	if (stub_flash_erase(bp_flash_addr, insn_sz) != ESP_XTENSA_STUB_ERR_OK) {
		STUB_LOGE("Failed to erase insn sector!\n");
		return ESP_XTENSA_STUB_ERR_FAIL;
	}
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 0] = insn[0];
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 1] = insn[1];
	if (insn_sz == 3)
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 2] = insn[2];
	rc = esp_rom_spiflash_write(bp_flash_addr & ~(STUB_FLASH_SECTOR_SIZE - 1),
		(uint32_t *)insn_sect,
		STUB_BP_INSN_SECT_BUF_SIZE);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to restore insn (%d)!\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}

	stub_flash_cache_flush();

#if STUB_LOG_LOCAL_LEVEL == STUB_LOG_VERBOSE
	uint8_t tmp[8];
	rc = esp_rom_spiflash_read(bp_flash_addr & ~0x3UL, (uint32_t *)tmp, sizeof(tmp));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read insn (%d)!\n", rc);
		return ESP_XTENSA_STUB_ERR_FAIL;
	}
	STUB_LOGD("%s: WROTE 0x%x 0x%x [%02x %02x %02x %02x %02x %02x %02x %02x]\n",
		__func__,
		bp_flash_addr,
		insn_buf_addr,
		tmp[0],
		tmp[1],
		tmp[2],
		tmp[3],
		tmp[4],
		tmp[5],
		tmp[6],
		tmp[7]);
#endif
	return ESP_XTENSA_STUB_ERR_OK;
}

static int stub_flash_handler(int cmd, va_list ap)
{
	int ret = ESP_XTENSA_STUB_ERR_OK;
	struct stub_flash_state flash_state;
	uint32_t arg1 = va_arg(ap, uint32_t);	/* flash_addr, start_sect */
	uint32_t arg2 = va_arg(ap, uint32_t);	/* size, number of sectors */
	uint8_t *arg3 = va_arg(ap, uint8_t *);	/* down_buf_addr, sectorts' state buf address */
	uint32_t arg4 = va_arg(ap, uint32_t);	/* down buf size */

	STUB_LOGD("%s a %x, s %d\n", __func__, arg1, arg2);

	stub_flash_state_prepare(&flash_state);

	uint32_t flash_size = stub_flash_get_size();
	if (flash_size == 0) {
		STUB_LOGE("Failed to get flash size!\n");
		ret = cmd == ESP_XTENSA_STUB_CMD_FLASH_SIZE ? 0 : ESP_XTENSA_STUB_ERR_FAIL;
		goto _flash_end;
	}
	if (cmd == ESP_XTENSA_STUB_CMD_FLASH_SIZE) {
		ret = flash_size;
		goto _flash_end;
	}
	esp_rom_spiflash_config_param(g_rom_flashchip.device_id,
		flash_size,
		STUB_FLASH_BLOCK_SIZE,
		STUB_FLASH_SECTOR_SIZE,
		STUB_FLASH_PAGE_SIZE,
		STUB_FLASH_STATUS_MASK);

	esp_rom_spiflash_result_t rc = esp_rom_spiflash_unlock();
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to unlock flash (%d)\n", rc);
		ret = ESP_XTENSA_STUB_ERR_FAIL;
		goto _flash_end;
	}

	switch (cmd) {
		case ESP_XTENSA_STUB_CMD_FLASH_READ:
			ret = stub_flash_read(arg1, arg2);
			break;
		case ESP_XTENSA_STUB_CMD_FLASH_ERASE:
			ret = stub_flash_erase(arg1, arg2);
			break;
		case ESP_XTENSA_STUB_CMD_FLASH_ERASE_CHECK:
			ret = stub_flash_erase_check(arg1, arg2, arg3);
			break;
		case ESP_XTENSA_STUB_CMD_FLASH_WRITE:
			ret = stub_flash_write(arg1, arg2, arg3, arg4);
			break;
		case ESP_XTENSA_STUB_CMD_FLASH_MAP_GET:
			ret = stub_flash_get_map(arg1, arg2);
			break;
		case ESP_XTENSA_STUB_CMD_FLASH_BP_SET:
			ret = stub_flash_set_bp(arg1, arg2, arg3);
			break;
		case ESP_XTENSA_STUB_CMD_FLASH_BP_CLEAR:
			ret = stub_flash_clear_bp(arg1, arg2, arg3);
			break;
#if STUB_DEBUG
		case ESP_XTENSA_STUB_CMD_FLASH_TEST:
			ret = stub_flash_test();
			break;
#endif
		default:
			ret = ESP_XTENSA_STUB_ERR_NOT_SUPPORTED;
	}

_flash_end:
	stub_flash_state_restore(&flash_state);
	return ret;
}

void ets_update_cpu_frequency(uint32_t ticks_per_us)
{
	/* do nothing for stub */
}

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
static void uart_console_configure(void)
{
	uartAttach();
	ets_install_uart_printf();
	/* Set configured UART console baud rate */
	uart_div_modify(CONFIG_CONSOLE_UART_NUM,
		(rtc_clk_apb_freq_get() << 4) / CONFIG_CONSOLE_UART_BAUDRATE);
}
#endif

int stub_main(int cmd, ...)
{
	va_list ap;
	int ret = 0;

	/* zero bss */
	for (uint32_t *p = &_bss_start; p < &_bss_end; p++)
		*p = 0;

	/* we get here just after OpenOCD's stub trampoline
	 * up to 5 parameters are passed via registers by that jumping code */
	/* interrupts level in PS is set to 5 to allow high prio IRQs only (including Debug
	 * Interrupt) */
	/* We need Debug Interrupt Level to allow breakpoints handling by OpenOCD */
#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
	stub_clock_configure();
	uart_console_configure();
#endif
	STUB_LOGD("BSS 0x%x..0x%x\n", &_bss_start, &_bss_end);
	STUB_LOGD("cmd %d\n", cmd);

	va_start(ap, cmd);
	if (cmd <= ESP_XTENSA_STUB_CMD_FLASH_MAX_ID)
		ret = stub_flash_handler(cmd, ap);
	else
		switch (cmd) {
#if STUB_DEBUG
			case ESP_XTENSA_STUB_CMD_TEST:
				STUB_LOGD("TEST %d\n", cmd);
				break;
#endif
			default:
				ret = ESP_XTENSA_STUB_ERR_NOT_SUPPORTED;
		}
	va_end(ap);

	STUB_LOGD("exit %d\n", ret);
	return ret;
}
