// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP flasher stub                                                      *
 *   Copyright (C) 2017-2019 Espressif Systems Ltd.                        *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 ***************************************************************************/
#include <stdarg.h>
#include <string.h>

#include <esp_app_trace.h>
#include <esp_app_trace_port.h>
#include <esp_flash_partitions.h>
#include <esp_image_format.h>

#include <stub_flasher_chip.h>

#include "stub_flasher.h"
#include "stub_logger.h"
#include "stub_flasher_int.h"

#include "mcu_boot.h"

#if STUB_LOG_ENABLE == 1
	#define STUB_BENCH(var)	\
	uint64_t var = stub_get_time()
#else
	#define STUB_BENCH(var)	\
	do {} while (0)
#endif

#define STUB_BP_INSN_SECT_BUF_SIZE        (2 * STUB_FLASH_SECTOR_SIZE)

#define CHECKSUM_ALIGN        16
#define IS_PADD(addr) ((addr) == 0)
#define IS_DRAM(addr) ((addr) >= SOC_DRAM_LOW && (addr) < SOC_DRAM_HIGH)
#define IS_IRAM(addr) ((addr) >= SOC_IRAM_LOW && (addr) < SOC_IRAM_HIGH)
#define IS_IROM(addr) ((addr) >= SOC_IROM_LOW && (addr) < SOC_IROM_HIGH)
#define IS_DROM(addr) ((addr) >= SOC_DROM_LOW && (addr) < SOC_DROM_HIGH)
#define IS_SRAM(addr) (IS_IRAM(addr) || IS_DRAM(addr))
#define IS_MMAP(addr) (IS_IROM(addr) || IS_DROM(addr))
#if SOC_RTC_FAST_MEM_SUPPORTED
# define IS_RTC_FAST_IRAM(addr) \
					((addr) >= SOC_RTC_IRAM_LOW && (addr) < SOC_RTC_IRAM_HIGH)
#define IS_RTC_FAST_DRAM(addr) \
					((addr) >= SOC_RTC_DRAM_LOW && (addr) < SOC_RTC_DRAM_HIGH)
#else
#define IS_RTC_FAST_IRAM(addr) 0
#define IS_RTC_FAST_DRAM(addr) 0
#endif
#if SOC_RTC_SLOW_MEM_SUPPORTED
#define IS_RTC_SLOW_DRAM(addr) \
					((addr) >= SOC_RTC_DATA_LOW && (addr) < SOC_RTC_DATA_HIGH)
#else
#define IS_RTC_SLOW_DRAM(addr) 0
#endif

#if SOC_MEM_TCM_SUPPORTED
#define IS_TCM(addr) ((addr) >= SOC_TCM_LOW && (addr) < SOC_TCM_HIGH)
#else
#define IS_TCM(addr) 0
#endif

#define IS_NONE(addr) (!IS_IROM(addr) && !IS_DROM(addr) \
					&& !IS_IRAM(addr) && !IS_DRAM(addr) \
					&& !IS_RTC_FAST_IRAM(addr) && !IS_RTC_FAST_DRAM(addr) \
					&& !IS_RTC_SLOW_DRAM(addr) && !IS_TCM(addr) \
					&& !IS_PADD(addr))

#define MAX_SEGMENT_COUNT 16

extern void stub_sha256_start(void);
extern void stub_sha256_data(const void *data, size_t data_len);
extern void stub_sha256_finish(uint8_t *digest);

extern uint32_t _bss_start;
extern uint32_t _bss_end;
extern uint32_t _data_start;
extern uint32_t _data_end;

/* g_ticks_us defined in ROMs for PRO and APP CPU */
extern uint32_t g_ticks_per_us_pro;

/* Flash data defined in ROM*/
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
extern esp_rom_spiflash_chip_t g_rom_flashchip;
#else
extern esp_rom_spiflash_legacy_data_t *rom_spiflash_legacy_data;
#define g_rom_flashchip (rom_spiflash_legacy_data->chip)
#endif

static struct {
	/* offset of next flash write */
	uint32_t next_write;
	/* number of output bytes remaining to write */
	uint32_t remaining_uncompressed;
	/* number of compressed bytes remaining to read */
	uint32_t remaining_compressed;
	/* inflator state for deflate write */
	tinfl_decompressor *inflator;
	/* unzip buffer */
	uint8_t *out_buf;
	/* */
	uint8_t *next_out;
} s_fs;

static uint32_t s_encrypt_binary;

const struct esp_flash_stub_desc __attribute__((section(".stub_desc")))  s_esp_stub_desc = {
	.magic = ESP_STUB_FLASHER_MAGIC_NUM,
	.stub_version = ESP_STUB_FLASHER_VERSION,
#ifdef CMD_FLASH_IDF_BINARY
	.idf_key = ESP_STUB_FLASHER_IDF_KEY,
#else
	.idf_key = 0x00,
#endif
};

/* used in app trace module */
uint32_t esp_log_early_timestamp(void)
{
	return 0;
}

/* used in app trace module */
uint32_t esp_clk_cpu_freq(void)
{
	return g_ticks_per_us_pro * MHZ;
}

void __assert_func(const char *path, int line, const char *func, const char *msg)
{
	STUB_LOGE("ASSERT at %s:%d '%s'\n", func, line, msg);
	while (1)
		;
}

void abort(void)
{
	STUB_LOGE("ABORT\n");
	while (1)
		;
}

/* used in REGI2C_WRITE and REGI2C_WRITE_MASK */
BaseType_t xPortEnterCriticalTimeout(portMUX_TYPE *mux, BaseType_t timeout)
{
	return (BaseType_t)1;
}

#if CMD_FLASH_TEST
static int stub_flash_test(void)
{
	int ret = ESP_STUB_ERR_OK;
	uint8_t buf[32] = { 9, 1, 2, 3, 4, 5, 6, 8 };
	uint32_t flash_addr = 0x1d4000;

	esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_sector(flash_addr / STUB_FLASH_SECTOR_SIZE);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to erase flash (%d)\n", rc);
		return ESP_STUB_ERR_FAIL;
	}

	rc = esp_rom_spiflash_write(flash_addr, (uint32_t *)buf, sizeof(buf));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to write flash (%d)\n", rc);
		return ESP_STUB_ERR_FAIL;
	}

	rc = esp_rom_spiflash_read(flash_addr, (uint32_t *)buf, sizeof(buf));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read flash (%d)\n", rc);
		return ESP_STUB_ERR_FAIL;
	}

	STUB_LOGD("Data: ");
	for (int i = 0; i < 10; i++)
		STUB_LOGD("%x ", buf[i]);
	STUB_LOGD("\n");

	return ret;
}
#endif

static int stub_apptrace_init(void)
{
	STUB_LOGI("Init apptrace module\n");
	esp_err_t err = esp_apptrace_init();
	if (err != ESP_OK) {
		STUB_LOGE("Failed to init apptrace module (%d)!\n", err);
		return ESP_STUB_ERR_FAIL;
	}
	return stub_apptrace_prepare();
}

static __maybe_unused int stub_flash_calc_hash(uint32_t addr, uint32_t size, uint8_t *hash)
{
	esp_rom_spiflash_result_t rc;
	uint32_t rd_cnt = 0, rd_sz = 0;
	uint8_t read_buf[ESP_STUB_RDWR_BUFF_SIZE];

	STUB_LOGD("%s %d bytes @ 0x%x\n", __func__, size, addr);

	stub_sha256_start();

	while (size > 0) {
		rd_sz = MIN(ESP_STUB_RDWR_BUFF_SIZE, size);
		rc = stub_flash_read_buff(addr + rd_cnt, (uint32_t *)read_buf, rd_sz);
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read flash (%d)!\n", rc);
			return ESP_STUB_ERR_FAIL;
		}
		stub_sha256_data(read_buf, rd_sz);
		size -= rd_sz;
		rd_cnt += rd_sz;
	}

	stub_sha256_finish(hash);

	STUB_LOGD("hash: %x%x%x...%x%x%x\n",
		hash[0], hash[1], hash[2],
		hash[29], hash[30], hash[31]);

	return ESP_STUB_ERR_OK;
}

static __maybe_unused int stub_flash_read(uint32_t addr, uint32_t size)
{
	esp_rom_spiflash_result_t rc;
	uint32_t total_cnt = 0;

#if CONFIG_STUB_STACK_DATA_POOL_SIZE > 0
	/* stub_apptrace_init alloc up buffers on stack */
	uint8_t stack_data_pool[CONFIG_STUB_STACK_DATA_POOL_SIZE];
	stub_stack_data_pool_init(stack_data_pool, sizeof(stack_data_pool));
#endif

	int ret = stub_apptrace_init();
	if (ret != ESP_STUB_ERR_OK)
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

		STUB_BENCH(start);
		uint8_t *buf = esp_apptrace_buffer_get(ESP_APPTRACE_DEST_TRAX,
			rd_sz,
			ESP_APPTRACE_TMO_INFINITE);
		if (!buf) {
			STUB_LOGE("Failed to get trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		STUB_BENCH(end);
		STUB_LOGD("Got trace buf %d bytes @ 0x%x in %lld us\n", rd_sz, buf, end - start);
		{
			STUB_BENCH(start);
			rc = stub_flash_read_buff(addr + total_cnt, (uint32_t *)buf, rd_sz);
			STUB_BENCH(end);
			STUB_LOGD("Read flash @ 0x%x sz %d in %d ms\n",
				addr + total_cnt,
				rd_sz,
				(end - start) / 1000);
		}

		/* regardless of the read result, first free the buffer */
		esp_err_t err = esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX,
			buf,
			ESP_APPTRACE_TMO_INFINITE);

		/* now check the read result */
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read flash (%d)!\n", rc);
			return ESP_STUB_ERR_FAIL;
		}

		if (err != ESP_OK) {
			STUB_LOGE("Failed to put trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}

		total_cnt += rd_sz;

		STUB_LOGD("Flush trace buf %d bytes @ 0x%x [%x %x %x %x %x %x %x %x]\n",
			rd_sz, buf, buf[-4], buf[-3], buf[-2], buf[-1],
			buf[0], buf[1], buf[2], buf[3]);
		err = esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to flush trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		STUB_LOGD("Sent trace buf %d bytes @ 0x%x\n", rd_sz, buf);
	}

	if (total_cnt < size) {
		if ((size - total_cnt) >= 4) {
			STUB_LOGE("Exited loop when remaing data size is more the 4 bytes!\n");
			return ESP_STUB_ERR_FAIL;	/*should never get here*/
		}
		/* if we exited loop because remaing data size is less than 4 bytes */
		uint8_t last_bytes[4];
		rc = stub_flash_read_buff(addr + total_cnt, (uint32_t *)last_bytes, 4);
		STUB_LOGD("Read padded word from flash @ 0x%x\n", addr + total_cnt);
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read last word from flash (%d)!\n", rc);
			return ESP_STUB_ERR_FAIL;
		}
		uint8_t *buf = esp_apptrace_buffer_get(ESP_APPTRACE_DEST_TRAX,
			size - total_cnt,
			ESP_APPTRACE_TMO_INFINITE);
		if (!buf) {
			STUB_LOGE("Failed to get trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		memcpy(buf, last_bytes, size - total_cnt);
		esp_err_t err = esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX,
			buf,
			ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to put trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		STUB_LOGD("Flush trace buf %d bytes @ 0x%x [%x %x %x %x %x %x %x %x]\n",
			size - total_cnt, buf, buf[-4], buf[-3], buf[-2], buf[-1],
			buf[0], buf[1], buf[2], buf[3]);
		err = esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
		if (err != ESP_OK) {
			STUB_LOGE("Failed to flush trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		STUB_LOGE("Sent last trace buf %d bytes @ 0x%x\n", size - total_cnt, buf);
	}
	STUB_LOGD("Read %d bytes @ 0x%x\n", size, addr);

	return ESP_STUB_ERR_OK;
}

static esp_rom_spiflash_result_t stub_spiflash_write(uint32_t spi_flash_addr, uint32_t *data, uint32_t len,
	bool encrypt)
{
	esp_rom_spiflash_result_t rc;

	/*
	We can ask hardware to encrypt binary as long as flash encryption is enabled and encrypt option is set.
	During breakpoint set and clear, we will set 'encrypt' flag according to efuse flash encryption settings.
	So that hardware will handle the encryption if necessary.
	During flash programming, 'encrypt' flag will reflect the user specified 'encrypt_binary' programming option.
	If this is not set, we assume the binary is encrypted by the user.
	If this is set but flash encryption is not enabled, by returning an error,
	we will warn the user to do the right operation.
	*/
	if (encrypt && stub_get_flash_encryption_mode() == ESP_FLASH_ENC_MODE_DISABLED)
		return ESP_STUB_ERR_FAIL;

	bool write_encrypted = stub_get_flash_encryption_mode() != ESP_FLASH_ENC_MODE_DISABLED && encrypt;

	STUB_BENCH(start);
	if (write_encrypted)
		rc = esp_rom_spiflash_write_encrypted(spi_flash_addr, data, len);
	else
		rc = esp_rom_spiflash_write(spi_flash_addr, data, len);
	STUB_BENCH(end);

	STUB_LOGD("Write %sflash @ 0x%x sz %d in %lld us\n",
		write_encrypted ? "encrypted-" : "",
		spi_flash_addr,
		len,
		end - start);

	return rc;
}

static int stub_write_aligned_buffer(void *data_buf, uint32_t length)
{
	while (length > 0 && s_fs.remaining_uncompressed > 0) {
		uint32_t out_bytes = length;
		uint32_t bytes_in_out_buf = s_fs.next_out - s_fs.out_buf;

		if (out_bytes + bytes_in_out_buf > ESP_STUB_UNZIP_BUFF_SIZE)
			out_bytes = ESP_STUB_UNZIP_BUFF_SIZE - bytes_in_out_buf;

		memcpy(s_fs.out_buf + bytes_in_out_buf, data_buf, out_bytes);

		bytes_in_out_buf += out_bytes;
		length -= out_bytes;
		data_buf += out_bytes;
		s_fs.next_out += out_bytes;

		if (s_fs.remaining_uncompressed - bytes_in_out_buf == 0 ||
			bytes_in_out_buf == ESP_STUB_UNZIP_BUFF_SIZE) {
			uint32_t wr_sz = bytes_in_out_buf;

			/* stub_spiflash_write() expects length to be aligned to 4 bytes.
			   If this is the last package we do not need to care about it here because OpenOCD flash driver
			   ensures that address and size are always aligned to sector size which is multiple of 4 */

			/* write buffer with aligned size */
			esp_rom_spiflash_result_t rc = stub_spiflash_write(s_fs.next_write, (uint32_t *)s_fs.out_buf,
				wr_sz, s_encrypt_binary);

			if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
				STUB_LOGE("Failed to write flash (%d)\n", rc);
				return ESP_STUB_ERR_FAIL;
			}

			/* change counters using unpadding len */
			s_fs.next_write += bytes_in_out_buf;
			s_fs.remaining_uncompressed -= bytes_in_out_buf;
			s_fs.next_out = s_fs.out_buf;
		}
	}

	return ESP_STUB_ERR_OK;
}

static __maybe_unused int stub_flash_write(void *args)
{
	uint32_t total_cnt = 0;
	uint8_t *buf = NULL;
	struct esp_flash_stub_flash_write_args *wargs = (struct esp_flash_stub_flash_write_args *)args;

	STUB_LOGD("Start writing %d bytes @ 0x%x opt %x\n", wargs->size, wargs->start_addr, wargs->options);

	s_encrypt_binary = wargs->options & ESP_STUB_FLASH_ENCRYPT_BINARY ? 1 : 0;

#if CONFIG_STUB_STACK_DATA_POOL_SIZE > 0
	/* for non-xtensa chips stub_apptrace_init alloc up buffers on stack, xtensa chips uses TRAX memory */
	uint8_t stack_data_pool[CONFIG_STUB_STACK_DATA_POOL_SIZE];
	stub_stack_data_pool_init(stack_data_pool, sizeof(stack_data_pool));
#endif

	int ret = stub_apptrace_init();
	if (ret != ESP_STUB_ERR_OK)
		return ret;
	STUB_LOGI("Init apptrace module down buffer %d bytes @ 0x%x\n",
		wargs->down_buf_size,
		wargs->down_buf_addr);
	esp_apptrace_down_buffer_config((uint8_t *)wargs->down_buf_addr, wargs->down_buf_size);

	uint8_t out_buf[ESP_STUB_UNZIP_BUFF_SIZE];
	s_fs.next_write = wargs->start_addr;
	s_fs.remaining_uncompressed = wargs->total_size;
	s_fs.out_buf = out_buf;
	s_fs.next_out = out_buf;

	while (total_cnt < wargs->size) {
		uint32_t wr_sz = wargs->size - total_cnt;
		STUB_LOGV("Req trace down buf %d bytes %d-%d\n",
			wr_sz,
			wargs->size,
			total_cnt);
		STUB_BENCH(start);
		buf = esp_apptrace_down_buffer_get(ESP_APPTRACE_DEST_TRAX,
			&wr_sz,
			ESP_APPTRACE_TMO_INFINITE);
		if (!buf) {
			STUB_LOGE("Failed to get trace down buf!\n");
			return ESP_STUB_ERR_FAIL;
		}

		STUB_BENCH(end);
		STUB_LOGV("Got trace down buf %d bytes @ 0x%x in %lld us\n", wr_sz, buf,
			end - start);

		ret = stub_write_aligned_buffer(buf, wr_sz);

		/* regardless of the write result, first free the buffer */
		esp_err_t err = esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX,
			buf,
			ESP_APPTRACE_TMO_INFINITE);

		if (err != ESP_OK) {
			STUB_LOGE("Failed to put trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		/* now check the write result */
		if (ret != ESP_STUB_ERR_OK)
			return ESP_STUB_ERR_INFLATE;

		total_cnt += wr_sz;
	}

	STUB_LOGD("Wrote %d bytes @ 0x%x\n", wargs->total_size, wargs->start_addr);

	return ESP_STUB_ERR_OK;
}

static int stub_write_inflated_data(void *data_buf, uint32_t length)
{
	if (length > s_fs.remaining_uncompressed) {
		/* Trim the final block, as it may have padding beyond
		    the length we are writing */
		length = s_fs.remaining_uncompressed;
	}

	if (length == 0)
		return ESP_STUB_ERR_OK;

	/* if this is the last package */
	if (length < ESP_STUB_UNZIP_BUFF_SIZE) {
		/* stub_spiflash_write() expects length to be aligned to 4 bytes.
		   If this is the last package we do not need to care about it here because OpenOCD flash driver
		   ensures that address and size are always aligned to sector size which is multiple of 4 */
		if (s_fs.remaining_uncompressed - length != 0) {
			STUB_LOGE("Unaligned offset! %d-%d\n", length, s_fs.remaining_uncompressed);
			return ESP_STUB_ERR_FAIL;
		}
	}

	/* write buffer with aligned size */
	esp_rom_spiflash_result_t rc = stub_spiflash_write(s_fs.next_write, (uint32_t *)data_buf, length,
		s_encrypt_binary);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to write flash (%d)\n", rc);
		return ESP_STUB_ERR_FAIL;
	}

	return ESP_STUB_ERR_OK;
}

static int stub_run_inflator(void *data_buf, uint32_t length)
{
	int status = TINFL_STATUS_NEEDS_MORE_INPUT;

	while (length > 0 && s_fs.remaining_uncompressed > 0 && status > TINFL_STATUS_DONE) {
		/* input remaining */
		size_t in_bytes = length;
		/* output space remaining */
		size_t out_bytes = s_fs.out_buf + ESP_STUB_UNZIP_BUFF_SIZE - s_fs.next_out;
		int flags = 0;

		if (s_fs.remaining_compressed > length)
			flags |= TINFL_FLAG_HAS_MORE_INPUT;

		STUB_BENCH(start);
		status = tinfl_decompress(s_fs.inflator, data_buf, &in_bytes,
			s_fs.out_buf, s_fs.next_out, &out_bytes,
			flags);
		STUB_BENCH(end);
		STUB_LOGV("tinfl_decompress in(%d) out(%d) (%lld)us\n",
			in_bytes,
			out_bytes,
			end - start);

		s_fs.remaining_compressed -= in_bytes;
		length -= in_bytes;
		data_buf += in_bytes;
		s_fs.next_out += out_bytes;
		size_t bytes_in_out_buf = s_fs.next_out - s_fs.out_buf;

		if (status <= TINFL_STATUS_DONE ||
			bytes_in_out_buf == ESP_STUB_UNZIP_BUFF_SIZE) {
			/* Output buffer full, or done */
			if (stub_write_inflated_data(s_fs.out_buf,
					bytes_in_out_buf) != ESP_STUB_ERR_OK)
				return ESP_STUB_ERR_FAIL;

			s_fs.next_write += bytes_in_out_buf;
			s_fs.remaining_uncompressed -= bytes_in_out_buf;
			s_fs.next_out = s_fs.out_buf;
		}
	}	/* while */

	if (status < TINFL_STATUS_DONE) {
		STUB_LOGE("Failed to inflate data (%d)\n", status);
		return ESP_STUB_ERR_INFLATE;
	}
	if (status == TINFL_STATUS_DONE && s_fs.remaining_uncompressed > 0) {
		STUB_LOGE("Not enough compressed data (%d)\n", s_fs.remaining_uncompressed);
		return ESP_STUB_ERR_NOT_ENOUGH_DATA;
	}
	if (status != TINFL_STATUS_DONE && s_fs.remaining_uncompressed == 0) {
		STUB_LOGE("Too much compressed data (%d)\n", length);
		return ESP_STUB_ERR_TOO_MUCH_DATA;
	}
	return ESP_STUB_ERR_OK;
}

static __maybe_unused int stub_flash_write_deflated(void *args)
{
	uint32_t total_cnt = 0;
	uint8_t *buf = NULL;
	struct esp_flash_stub_flash_write_args *wargs = (struct esp_flash_stub_flash_write_args *)args;

	STUB_LOGD("Start writing %d bytes @ 0x%x opt %x\n", wargs->size, wargs->start_addr, wargs->options);

	s_encrypt_binary = wargs->options & ESP_STUB_FLASH_ENCRYPT_BINARY ? 1 : 0;

#if CONFIG_STUB_STACK_DATA_POOL_SIZE > 0
	/* for non-xtensa chips stub_apptrace_init alloc up buffers on stack, xtensa chips uses TRAX
	 *memory */
	uint8_t stack_data_pool[CONFIG_STUB_STACK_DATA_POOL_SIZE];
	stub_stack_data_pool_init(stack_data_pool, sizeof(stack_data_pool));
#endif

	int ret = stub_apptrace_init();
	if (ret != ESP_STUB_ERR_OK)
		return ret;

	STUB_LOGI("Init apptrace module down buffer %d bytes @ 0x%x\n",
		wargs->down_buf_size,
		wargs->down_buf_addr);
	esp_apptrace_down_buffer_config((uint8_t *)wargs->down_buf_addr, wargs->down_buf_size);

	STUB_LOGI("Uncompressed data size %d bytes\n", wargs->total_size);

	/* both of them must be in stack! */
	tinfl_decompressor inflator;
	uint8_t out_buf[ESP_STUB_UNZIP_BUFF_SIZE];
	/* */

	s_fs.next_write = wargs->start_addr;
	s_fs.remaining_uncompressed = wargs->total_size;
	s_fs.remaining_compressed = wargs->size;
	s_fs.inflator = &inflator;
	s_fs.out_buf = out_buf;
	s_fs.next_out = out_buf;
	tinfl_init(s_fs.inflator);

	while (total_cnt < wargs->size) {
		uint32_t wr_sz = wargs->size - total_cnt;
		STUB_LOGV("Req trace down buf %d bytes %d-%d\n", wr_sz, wargs->size, total_cnt);

		STUB_BENCH(start);
		buf = esp_apptrace_down_buffer_get(ESP_APPTRACE_DEST_TRAX, &wr_sz, ESP_APPTRACE_TMO_INFINITE);
		if (!buf) {
			STUB_LOGE("Failed to get trace down buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		STUB_BENCH(end);
		STUB_LOGV("Got trace down buf %d bytes @ 0x%x in %lld us\n", wr_sz, buf, end - start);

		if (stub_run_inflator(buf, wr_sz) != ESP_STUB_ERR_OK)
			return ESP_STUB_ERR_INFLATE;

		/* regardless of the write result, first free the buffer */
		esp_err_t err = esp_apptrace_down_buffer_put(ESP_APPTRACE_DEST_TRAX,
			buf,
			ESP_APPTRACE_TMO_INFINITE);

		if (err != ESP_OK) {
			STUB_LOGE("Failed to put trace buf!\n");
			return ESP_STUB_ERR_FAIL;
		}
		/* now check the write result */
		if (ret != ESP_STUB_ERR_OK)
			return ESP_STUB_ERR_INFLATE;

		total_cnt += wr_sz;
	}

	STUB_LOGD("Wrote %d bytes @ 0x%x\n", wargs->total_size, wargs->start_addr);

	return ESP_STUB_ERR_OK;
}

esp_rom_spiflash_result_t esp_rom_spiflash_erase_area(uint32_t start_addr, uint32_t area_len)
{
	int32_t total_sector_num;
	int32_t head_sector_num;
	uint32_t sector_no;
	uint32_t sector_num_per_block;

	/* set read mode to Fastmode ,not QDIO mode for erase
	 *
	 * TODO: this is probably a bug as it doesn't re-enable QIO mode, not serious as this
	 * function is not used in IDF.
	 * esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_SLOWRD_MODE); */

	/* check if area is oversize of flash */
	if (start_addr + area_len > g_rom_flashchip.chip_size)
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	/* start_addr is aligned as sector boundary */
	if (start_addr % g_rom_flashchip.sector_size != 0)
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	/* Unlock flash to enable erase */
	if (esp_rom_spiflash_unlock(/*&g_rom_flashchip*/) != ESP_ROM_SPIFLASH_RESULT_OK)
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	sector_no = start_addr / g_rom_flashchip.sector_size;
	sector_num_per_block = g_rom_flashchip.block_size / g_rom_flashchip.sector_size;
	total_sector_num = area_len % g_rom_flashchip.sector_size == 0 ?
		area_len / g_rom_flashchip.sector_size :
		1 + (area_len / g_rom_flashchip.sector_size);
	/* check if erase area reach over block boundary */
	head_sector_num = sector_num_per_block - (sector_no % sector_num_per_block);
	head_sector_num = head_sector_num >= total_sector_num ? total_sector_num : head_sector_num;

	/* JJJ, BUG of 6.0 erase
	 * middle part of area is aligned by blocks */
	total_sector_num -= head_sector_num;

	STUB_LOGD("tsn:%d hsn:%d sn:%d snpb:%d\n", total_sector_num, head_sector_num, sector_no, sector_num_per_block);

	/* head part of area is erased */
	while (head_sector_num > 0) {
		if (esp_rom_spiflash_erase_sector(sector_no) != ESP_ROM_SPIFLASH_RESULT_OK)
			return ESP_ROM_SPIFLASH_RESULT_ERR;
		sector_no++;
		head_sector_num--;
	}
	while (total_sector_num > sector_num_per_block) {
		if (ESP_ROM_SPIFLASH_RESULT_OK !=
			esp_rom_spiflash_erase_block(sector_no / sector_num_per_block))
			return ESP_ROM_SPIFLASH_RESULT_ERR;
		sector_no += sector_num_per_block;
		total_sector_num -= sector_num_per_block;
	}

	/* tail part of area burn */
	while (total_sector_num > 0) {
		if (esp_rom_spiflash_erase_sector(sector_no) != ESP_ROM_SPIFLASH_RESULT_OK)
			return ESP_ROM_SPIFLASH_RESULT_ERR;
		sector_no++;
		total_sector_num--;
	}

	return ESP_ROM_SPIFLASH_RESULT_OK;
}

static __maybe_unused int stub_flash_erase(uint32_t flash_addr, uint32_t size)
{
	int ret = ESP_STUB_ERR_OK;

	if (flash_addr & (STUB_FLASH_SECTOR_SIZE - 1))
		flash_addr &= ~(STUB_FLASH_SECTOR_SIZE - 1);

	if (size & (STUB_FLASH_SECTOR_SIZE - 1))
		size = (size + (STUB_FLASH_SECTOR_SIZE - 1)) & ~(STUB_FLASH_SECTOR_SIZE - 1);

	STUB_LOGD("erase flash @ 0x%x, sz %d\n", flash_addr, size);

	STUB_BENCH(start);
	esp_rom_spiflash_result_t rc = esp_rom_spiflash_erase_area(flash_addr, size);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to erase flash (%d)\n", rc);
		return ESP_STUB_ERR_FAIL;
	}
	STUB_BENCH(end);
	STUB_LOGD("Erased %d bytes @ 0x%x in %lld ms\n", size, flash_addr,
		(end - start) / 1000);
	return ret;
}

static __maybe_unused int stub_flash_erase_check(uint32_t start_sec, uint32_t sec_num, uint8_t *sec_erased)
{
	int ret = ESP_STUB_ERR_OK;
	uint8_t buf[STUB_FLASH_SECTOR_SIZE / 8];/* implying that sector size is multiple of sizeof(buf) */

	STUB_LOGD("erase check start %d, sz %d\n", start_sec, sec_num);

	for (int i = start_sec; i < start_sec + sec_num; i++) {
		sec_erased[i] = 1;
		for (int k = 0; k < STUB_FLASH_SECTOR_SIZE / sizeof(buf); k++) {
			esp_rom_spiflash_result_t rc = stub_flash_read_buff(i * STUB_FLASH_SECTOR_SIZE + k * sizeof(buf),
				(uint32_t *)buf,
				sizeof(buf));
			if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
				STUB_LOGE("Failed to read flash (%d)!\n", rc);
				return ESP_STUB_ERR_FAIL;
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
		case 0x12:
			size = 256 * 1024;
			break;
		case 0x13:
			size = 512 * 1024;
			break;
		case 0x14:
			size = 1 * 1024 * 1024;
			break;
		case 0x15:
			size = 2 * 1024 * 1024;
			break;
		case 0x16:
			size = 4 * 1024 * 1024;
			break;
		case 0x17:
			size = 8 * 1024 * 1024;
			break;
		case 0x18:
			size = 16 * 1024 * 1024;
			break;
		case 0x19:
			size = 32 * 1024 * 1024;
			break;
		case 0x20:
			size = 64 * 1024 * 1024;
			break;
		case 0x21:
			size = 128 * 1024 * 1024;
			break;
		case 0x22:
			size = 256 * 1024 * 1024;
			break;
		case 0x39:
			size = 32 * 1024 * 1024;
			break;
	default:
		size = 0;
	}
	STUB_LOGD("Flash ID %x, size %d KB\n", id, size / 1024);
	return size;
}

union stub_image_header {
	struct {
		uint8_t esp_magic;
		uint8_t segment_count;
	};
	uint32_t mcuboot_magic;
};

/* Mcu Boot Image Layout
 * mcuboot_hdr: org = 0x0,  len = 0x20 (magic 0x96f3b83d)
 * metadata:    org = 0x20, len = 0x60 (magic 0xace637d3)
 * FLASH:       org = 0x80, len = FLASH_SIZE - 0x80
*/
static int stub_flash_get_mcuboot_mappings(uint32_t off, struct esp_flash_mapping *flash_map)
{
	esp_program_header_t prg_hdr;
	esp_rom_spiflash_result_t rc = stub_flash_read_buff(off + MCU_BOOT_HEADER_SIZE,
		(uint32_t *)&prg_hdr, sizeof(prg_hdr));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read mcuboot header\n");
		return ESP_STUB_ERR_READ_APP_IMAGE_HEADER;
	}

	if (prg_hdr.header_magic != MCU_BOOT_PROGRAM_HEADER_MAGIC) {
		STUB_LOGE("Invalid magic number 0x%x\n", prg_hdr.header_magic);
		return ESP_STUB_ERR_INVALID_APP_MAGIC;
	}

	if (!IS_DROM(prg_hdr.drom_map_addr) || !IS_IROM(prg_hdr.irom_map_addr)) {
		STUB_LOGE("Invalid DROM/IROM addr (0x%x)/(0x%x)\n",
			prg_hdr.drom_map_addr, prg_hdr.irom_map_addr);
		return ESP_STUB_ERR_FAIL;
	}

	flash_map->maps[0].phy_addr = prg_hdr.drom_flash_offset + off;
	flash_map->maps[0].load_addr = prg_hdr.drom_map_addr;
	flash_map->maps[0].size = prg_hdr.drom_size;
	flash_map->maps[1].phy_addr = prg_hdr.irom_flash_offset + off;
	flash_map->maps[1].load_addr = prg_hdr.irom_map_addr;
	flash_map->maps[1].size = prg_hdr.irom_size;
	flash_map->maps_num = 2;

	for (unsigned int map_num = 0; map_num < 2; ++map_num)
		STUB_LOGI("Mapped segment %d: %d bytes @ 0x%x -> 0x%x\n",
			map_num,
			flash_map->maps[map_num].size,
			flash_map->maps[map_num].phy_addr,
			flash_map->maps[map_num].load_addr);

	return ESP_STUB_ERR_OK;
}

static int stub_flash_get_app_mappings(uint32_t off, struct esp_flash_mapping *flash_map)
{
	union stub_image_header stub_img_hdr;
	uint16_t maps_num = 0;
	bool padding_checksum = false;
	unsigned int ram_segments = 0;

	/* clear the flash mappings */
	for (int i = 0; i < ESP_STUB_FLASH_MAPPINGS_MAX_NUM; i++)
		flash_map->maps[i].load_addr = 0;

	esp_rom_spiflash_result_t rc = stub_flash_read_buff(off, &stub_img_hdr, sizeof(stub_img_hdr));
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read magic byte!\n");
		return ESP_STUB_ERR_READ_APP_IMAGE_HEADER;
	}

	if (stub_img_hdr.mcuboot_magic == MCU_BOOT_HEADER_MAGIC) {
		STUB_LOGI("Mcu boot header found\n");
		return stub_flash_get_mcuboot_mappings(off, flash_map);
	}

	if (stub_img_hdr.esp_magic != ESP_IMAGE_HEADER_MAGIC) {
		STUB_LOGE("Invalid magic number 0x%x\n", stub_img_hdr.esp_magic);
		return ESP_STUB_ERR_INVALID_APP_MAGIC;
	}

	STUB_LOGI("Found app image: magic 0x%x, %d segments\n",
		stub_img_hdr.esp_magic,
		stub_img_hdr.segment_count);

	uint32_t flash_addr = off + sizeof(esp_image_header_t);

	for (int k = 0; k < MAX_SEGMENT_COUNT; k++) {
		esp_image_segment_header_t seg_hdr;
		rc = stub_flash_read_buff(flash_addr, (uint32_t *)&seg_hdr, sizeof(seg_hdr));
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read app segment header (%d)!\n", rc);
			return ESP_STUB_ERR_READ_APP_SEGMENT;
		}
		STUB_LOGI("App segment %d: %d bytes @ 0x%x\n",
			k,
			seg_hdr.data_len,
			seg_hdr.load_addr);

		if (IS_NONE(seg_hdr.load_addr))
			break;

		if (IS_RTC_FAST_IRAM(seg_hdr.load_addr) || IS_RTC_FAST_DRAM(seg_hdr.load_addr) ||
			IS_RTC_SLOW_DRAM(seg_hdr.load_addr) || IS_TCM(seg_hdr.load_addr))
			ram_segments++;

		if (IS_MMAP(seg_hdr.load_addr)) {
			STUB_LOGI("Mapped segment %d: %d bytes @ 0x%x -> 0x%x\n",
				maps_num,
				seg_hdr.data_len,
				flash_addr + sizeof(seg_hdr),
				seg_hdr.load_addr);

			/*
			 * Make sure DROM mapping will be at the first index. (OpenOCD expects DROM mapping at index 0)
			 * Note: New targets have identical IROM/DROM address. Thats why additional check is needed.
			 */
			int index = IS_DROM(seg_hdr.load_addr) && !flash_map->maps[0].load_addr ? 0 : 1;
			flash_map->maps[index].phy_addr = flash_addr + sizeof(seg_hdr);
			flash_map->maps[index].load_addr = seg_hdr.load_addr;
			flash_map->maps[index].size = seg_hdr.data_len;
			maps_num++;

			if (maps_num >= ESP_STUB_FLASH_MAPPINGS_MAX_NUM)
				/* No need to read more. DROM, IROM mapping is done */
				break;
		}

		if (IS_SRAM(seg_hdr.load_addr))
			ram_segments++;

		flash_addr += sizeof(seg_hdr) + seg_hdr.data_len;
		if (ram_segments == stub_img_hdr.segment_count && !padding_checksum) {
			/* add padding */
			flash_addr += CHECKSUM_ALIGN - (flash_addr % CHECKSUM_ALIGN);
			padding_checksum = true;
		}
	}
	flash_map->maps_num = maps_num;
	return ESP_STUB_ERR_OK;
}

static __maybe_unused int stub_flash_get_map(uint32_t app_off, uint32_t maps_addr, uint32_t flash_size)
{
	esp_rom_spiflash_result_t rc;
	esp_partition_info_t part;
	struct esp_stub_flash_map *flash_map = (struct esp_stub_flash_map *)maps_addr;

	flash_map->flash_size = flash_size;
	if (flash_size == 0) {
		flash_map->retcode = ESP_STUB_ERR_FLASH_SIZE;
		return ESP_STUB_ERR_OK;
	}

	STUB_LOGD("%s: 0x%x 0x%x\n", __func__, app_off, maps_addr);
	flash_map->retcode = ESP_STUB_ERR_OK;
	flash_map->map.maps_num = 0;
	if (app_off != (uint32_t)-1) {
		flash_map->retcode = stub_flash_get_app_mappings(app_off, &flash_map->map);
		return ESP_STUB_ERR_OK;
	}

	for (uint32_t i = 0;; i++) {
		rc = stub_flash_read_buff(ESP_PARTITION_TABLE_OFFSET + i * sizeof(esp_partition_info_t),
				(uint32_t *)&part,
				sizeof(part));
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read partitions table entry (%d)!\n", rc);
			flash_map->retcode = ESP_STUB_ERR_READ_PARTITION;
			return ESP_STUB_ERR_OK;
		}
		if (part.magic != ESP_PARTITION_MAGIC) {
			STUB_LOGE("Invalid partition table magic! (0x%x)\n", part.magic);
			flash_map->retcode = ESP_STUB_ERR_INVALID_IMAGE;
			return ESP_STUB_ERR_OK;
		}
		if (part.pos.offset > flash_size || part.pos.offset + part.pos.size > flash_size) {
			STUB_LOGE("Partition %d invalid - offset 0x%x size 0x%x exceeds flash chip size 0x%x\n",
				i,
				part.pos.offset,
				part.pos.size,
				flash_size);
			flash_map->retcode = ESP_STUB_ERR_INVALID_PARTITION;
			return ESP_STUB_ERR_OK;
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
			flash_map->retcode = stub_flash_get_app_mappings(part.pos.offset, &flash_map->map);
			return ESP_STUB_ERR_OK;
		}
	}
	/* PART_TYPE_APP not found */
	return ESP_STUB_ERR_OK;
}

#define ALIGN_DOWN(x, a)        ((x) & ~((typeof(x))(a) - 1))

static size_t stub_get_inst_buff_size(uint32_t bp_flash_addr, uint8_t inst_size)
{
	int sector_no = ALIGN_DOWN(bp_flash_addr, STUB_FLASH_SECTOR_SIZE) / STUB_FLASH_SECTOR_SIZE;
	int next_sector_no = ALIGN_DOWN(bp_flash_addr + inst_size, STUB_FLASH_SECTOR_SIZE) / STUB_FLASH_SECTOR_SIZE;
	size_t buff_size = STUB_FLASH_SECTOR_SIZE;
	if (next_sector_no > sector_no)
		buff_size *= 2;
	STUB_LOGD("%s: %d %d 0x%x\n", __func__, sector_no, next_sector_no, buff_size);
	return buff_size;
}

/**
* Possible BP layouts in flash:
* 1) addr is aligned to 4 bytes (in 1 sector)
* 2) addr is unaligned to 4 bytes, BP is not crossing sector's boundary (in 1 sector)
*   - not crossing 4 bytes alignment boundary
*   - crossing 4 bytes alignment boundary
* 3) addr is unaligned to 4 bytes, BP is crossing sector's boundary (in 2 sectors)
*/
static __maybe_unused uint8_t stub_flash_set_bp(uint32_t bp_flash_addr,
	uint32_t insn_buf_addr, uint8_t *insn_sect)
{
	esp_rom_spiflash_result_t rc;
	const size_t inst_buff_size = stub_get_inst_buff_size(bp_flash_addr, stub_get_max_insn_size());

	STUB_LOGD("%s: 0x%x 0x%x\n", __func__, bp_flash_addr, insn_buf_addr);

	stub_flash_cache_flush();

	rc = stub_flash_read_buff(ALIGN_DOWN(bp_flash_addr, STUB_FLASH_SECTOR_SIZE),
		(uint32_t *)insn_sect,
		inst_buff_size);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read insn sector (%d)!\n", rc);
		return 0;
	}
	uint8_t insn_sz = stub_get_insn_size(&insn_sect[bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)]);
	memcpy((void *)insn_buf_addr, &insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1))], insn_sz);
	STUB_LOGI("Read insn [%02x %02x %02x %02x] %d bytes @ 0x%x\n",
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 0],
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 1],
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 2],
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 3],
		insn_sz,
		bp_flash_addr);

	/* this will erase full sector or two */
	if (stub_flash_erase(bp_flash_addr, inst_buff_size) != ESP_STUB_ERR_OK) {
		STUB_LOGE("Failed to erase insn sector!\n");
		return 0;
	}
	union {
		uint32_t d32;
		uint8_t d8[4];
	} break_insn;
	break_insn.d32 = stub_get_break_insn(insn_sz);
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 0] = break_insn.d8[0];
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 1] = break_insn.d8[1];
	if (insn_sz > 2)
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 2] = break_insn.d8[2];
	if (insn_sz > 3)
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 3] = break_insn.d8[3];
	rc = stub_spiflash_write(bp_flash_addr & ~(STUB_FLASH_SECTOR_SIZE - 1),
		(uint32_t *)insn_sect,
		inst_buff_size,
		stub_get_flash_encryption_mode() != ESP_FLASH_ENC_MODE_DISABLED);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to write break insn (%d)!\n", rc);
		return 0;
	}

	stub_flash_cache_flush();

#if STUB_LOG_ENABLE == 1
	if (stub_get_log_level() >= STUB_LOG_LEVEL_DEBUG) {
		uint8_t tmp[8];
		rc = stub_flash_read_buff(bp_flash_addr & ~0x3UL, (uint32_t *)tmp, sizeof(tmp));
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read insn (%d)!\n", rc);
			return ESP_STUB_ERR_FAIL;
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
	}
#endif
	return insn_sz;
}

/*
 * This function combines multiple flash addresses and instructions into a single buffer.
 * The following variables are used:
 * bp_flash_addr --> bp0_flash_addr + bp1_flash_addr + bp2_flash_addr + ... + bpn_flash_addr
 * insn_buf_addr <-- insn_sz0 + inst0 + insn_sz1 + inst1 + insn_sz2 + inst2 + ... + insn_szn + instn
 *
 * Each bp_flash_addr is 4 bytes long.
 * Each insn_sz is 1 byte long, representing the size of the corresponding instruction. (2 or 3)
 * Each inst is 3 bytes long.
 *
 */
static __maybe_unused uint8_t stub_flash_set_bp_multi(uint32_t *bp_flash_addr,
	uint8_t *insn_buf_addr, uint8_t *insn_sect, uint32_t num_bps)
{
	STUB_LOGD("%s %d bps\n", __func__, num_bps);

	struct esp_flash_stub_bp_instructions *bp_insts = (struct esp_flash_stub_bp_instructions *)insn_buf_addr;
	for (size_t i = 0; i < num_bps; ++i) {
		uint8_t rc = stub_flash_set_bp(bp_flash_addr[i], (uint32_t)&bp_insts[i].buff, insn_sect);
		if (rc == 0)
			return rc;
		bp_insts[i].size = rc;
	}

	return num_bps * sizeof(struct esp_flash_stub_bp_instructions);
}

static __maybe_unused int stub_flash_clear_bp(uint32_t bp_flash_addr,
	uint32_t insn_buf_addr, uint8_t *insn_sect)
{
	esp_rom_spiflash_result_t rc;
	uint8_t *insn = (uint8_t *)insn_buf_addr;
	uint8_t insn_sz = stub_get_insn_size(insn);
	const size_t inst_buff_size = stub_get_inst_buff_size(bp_flash_addr, insn_sz);

	STUB_LOGD("%s: 0x%x 0x%x [%02x %02x %02x %02x]\n",
		__func__,
		bp_flash_addr,
		insn_buf_addr,
		insn[0],
		insn[1],
		insn[2],
		insn[3]);

	stub_flash_cache_flush();

	rc = stub_flash_read_buff(ALIGN_DOWN(bp_flash_addr, STUB_FLASH_SECTOR_SIZE),
		(uint32_t *)insn_sect,
		inst_buff_size);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to read insn sector (%d)!\n", rc);
		return ESP_STUB_ERR_FAIL;
	}

	/* this will erase full sector or two */
	if (stub_flash_erase(bp_flash_addr, inst_buff_size) != ESP_STUB_ERR_OK) {
		STUB_LOGE("Failed to erase insn sector!\n");
		return ESP_STUB_ERR_FAIL;
	}
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 0] = insn[0];
	insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 1] = insn[1];
	if (insn_sz > 2)
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 2] = insn[2];
	if (insn_sz > 3)
		insn_sect[(bp_flash_addr & (STUB_FLASH_SECTOR_SIZE - 1)) + 3] = insn[3];
	rc = stub_spiflash_write(bp_flash_addr & ~(STUB_FLASH_SECTOR_SIZE - 1),
		(uint32_t *)insn_sect,
		inst_buff_size,
		stub_get_flash_encryption_mode() != ESP_FLASH_ENC_MODE_DISABLED);
	if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
		STUB_LOGE("Failed to restore insn (%d)!\n", rc);
		return ESP_STUB_ERR_FAIL;
	}

	stub_flash_cache_flush();

#if STUB_LOG_ENABLE == 1
	if (stub_get_log_level() >= STUB_LOG_LEVEL_DEBUG) {
		uint8_t tmp[8];
		rc = stub_flash_read_buff(bp_flash_addr & ~0x3UL, (uint32_t *)tmp, sizeof(tmp));
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to read insn (%d)!\n", rc);
			return ESP_STUB_ERR_FAIL;
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
	}
#endif
	return ESP_STUB_ERR_OK;
}

static __maybe_unused uint8_t stub_flash_clear_bp_multi(uint32_t *bp_flash_addr,
	uint8_t *insn_buf_addr, uint8_t *insn_sect, uint32_t num_bps)
{
	STUB_LOGD("%s %d bps\n", __func__, num_bps);

	struct esp_flash_stub_bp_instructions *bp_insts = (struct esp_flash_stub_bp_instructions *)insn_buf_addr;

	for (size_t i = 0; i < num_bps; ++i) {
		int rc = stub_flash_clear_bp(bp_flash_addr[i], (uint32_t)&bp_insts[i].buff, insn_sect);
		if (rc < 0)
			return rc;
	}

	return ESP_STUB_ERR_OK;
}

static int stub_flash_handler(int cmd, va_list ap)
{
	int ret = ESP_STUB_ERR_OK;
	struct stub_flash_state flash_state;
	uint32_t arg1 __maybe_unused = va_arg(ap, uint32_t);  /* flash_addr, start_sect */
	uint32_t arg2 __maybe_unused = va_arg(ap, uint32_t);	/* number of sectors */
	uint8_t *arg3 __maybe_unused = va_arg(ap, uint8_t *);	/* sectors' state buf address */
	uint32_t arg4 __maybe_unused = va_arg(ap, uint32_t);	/* set/clear bp count */

	STUB_LOGD("%s arg1 %x, arg2 %d\n", __func__, arg1, arg2);

	stub_flash_state_prepare(&flash_state);

	uint32_t flash_size = stub_flash_get_size();
	if (flash_size == 0) {
		STUB_LOGE("Failed to get flash size!\n");
		if (cmd != ESP_STUB_CMD_FLASH_MAP_GET) {
			ret = ESP_STUB_ERR_FAIL;
			goto _flash_end;
		}
		/* We will fill the esp_stub_flash_map struct and set the return code there */
	} else {
		esp_rom_spiflash_config_param(g_rom_flashchip.device_id,
			flash_size,
			STUB_FLASH_BLOCK_SIZE,
			STUB_FLASH_SECTOR_SIZE,
			STUB_FLASH_PAGE_SIZE,
			STUB_FLASH_STATUS_MASK);

		esp_rom_spiflash_result_t rc = esp_rom_spiflash_unlock();
		if (rc != ESP_ROM_SPIFLASH_RESULT_OK) {
			STUB_LOGE("Failed to unlock flash (%d)\n", rc);
			ret = ESP_STUB_ERR_FAIL;
			goto _flash_end;
		}
	}

	switch (cmd) {
#ifdef CMD_FLASH_READ
	case ESP_STUB_CMD_FLASH_READ:
		ret = stub_flash_read(arg1, arg2);
		break;
#endif
#ifdef CMD_FLASH_ERASE
	case ESP_STUB_CMD_FLASH_ERASE:
		ret = stub_flash_erase(arg1, arg2);
		break;
#endif
#ifdef CMD_FLASH_ERASE_CHECK
	case ESP_STUB_CMD_FLASH_ERASE_CHECK:
		ret = stub_flash_erase_check(arg1, arg2, arg3);
		break;
#endif
#ifdef CMD_FLASH_WRITE
	case ESP_STUB_CMD_FLASH_WRITE:
		ret = stub_flash_write((void *)arg1);
		break;
#endif
#ifdef CMD_FLASH_WRITE_DEFLATED
	case ESP_STUB_CMD_FLASH_WRITE_DEFLATED:
		ret = stub_flash_write_deflated((void *)arg1);
		break;
#endif
#ifdef CMD_FLASH_CALC_HASH
	case ESP_STUB_CMD_FLASH_CALC_HASH:
		ret = stub_flash_calc_hash(arg1, arg2, arg3);
		break;
#endif
#if defined(CMD_FLASH_MAP_GET) || defined(CMD_FLASH_MULTI_COMMAND) || defined(CMD_FLASH_IDF_BINARY)
	case ESP_STUB_CMD_FLASH_MAP_GET:
		ret = stub_flash_get_map(arg1, arg2, flash_size);
		break;
#endif
#if defined(CMD_FLASH_BP_SET) || defined(CMD_FLASH_MULTI_COMMAND) || defined(CMD_FLASH_IDF_BINARY)
	case ESP_STUB_CMD_FLASH_BP_SET:
		ret = stub_flash_set_bp_multi((void *)arg1, (void *)arg2, (void *)arg3, arg4);
		break;
#endif
#if defined(CMD_FLASH_BP_CLEAR) || defined(CMD_FLASH_MULTI_COMMAND) || defined(CMD_FLASH_IDF_BINARY)
	case ESP_STUB_CMD_FLASH_BP_CLEAR:
		ret = stub_flash_clear_bp_multi((void *)arg1, (void *)arg2, (void *)arg3, arg4);
		break;
#endif
#ifdef CMD_FLASH_CLOCK_CONFIGURE
	case ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE:
		ret = stub_cpu_clock_configure(arg1);
		if (stub_get_log_dest() == STUB_LOG_DEST_UART)
			stub_uart_console_configure(stub_get_log_dest());
		break;
#endif
#ifdef CMD_FLASH_TEST
	case ESP_STUB_CMD_FLASH_TEST:
		ret = stub_flash_test();
		break;
#endif
	default:
		ret = ESP_STUB_ERR_NOT_SUPPORTED;
	}

_flash_end:
	stub_flash_state_restore(&flash_state);
	return ret;
}

uint32_t stub_esp_clk_cpu_freq(void)
{
	extern uint32_t g_stub_cpu_freq_hz;
	return g_stub_cpu_freq_hz;
}

void ets_update_cpu_frequency(uint32_t ticks_per_us)
{
	/* do nothing for stub */
}

__attribute__((weak)) int stub_cpu_clock_configure(int cpu_freq_mhz)
{
	return 0;
}

__attribute__((weak)) void stub_uart_console_configure(int dest)
{
}

__attribute__((weak)) esp_flash_enc_mode_t stub_get_flash_encryption_mode(void)
{
	return ESP_FLASH_ENC_MODE_DISABLED;
}

__attribute__((weak)) void stub_log_init(enum stub_log_levels level, enum stub_log_destination dest)
{
}

#if STUB_LOG_ENABLE == 1
const char *cmd_to_str(int cmd)
{
	switch (cmd) {
	case ESP_STUB_CMD_FLASH_READ: return "FLASH_READ";
	case ESP_STUB_CMD_FLASH_WRITE: return "FLASH_WRITE";
	case ESP_STUB_CMD_FLASH_ERASE: return "FLASH_ERASE";
	case ESP_STUB_CMD_FLASH_ERASE_CHECK: return "FLASH_ERASE_CHECK";
	case ESP_STUB_CMD_FLASH_MAP_GET: return "FLASH_MAP_GET";
	case ESP_STUB_CMD_FLASH_BP_SET: return "FLASH_BP_SET";
	case ESP_STUB_CMD_FLASH_BP_CLEAR: return "FLASH_BP_CLEAR";
	case ESP_STUB_CMD_FLASH_TEST: return "FLASH_TEST";
	case ESP_STUB_CMD_FLASH_WRITE_DEFLATED: return "FLASH_WRITE_DEFLATED";
	case ESP_STUB_CMD_FLASH_CALC_HASH: return "FLASH_CALC_HASH";
	case ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE: return "CLOCK_CONFIGURE";
	default: return "";
	}
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
	/* interrupts level in PS is set to 5 to allow high prio IRQs only (including Debug Interrupt) */
	/* We need Debug Interrupt Level to allow breakpoints handling by OpenOCD */
	stub_log_init(STUB_RESET_LOG_LEVEL, STUB_RESET_LOG_DEST);
	STUB_LOGD("cpu_freq:%d Mhz\n", stub_esp_clk_cpu_freq() / MHZ);
	STUB_LOGD("DATA 0x%x..0x%x\n", &_data_start, &_data_end);
	STUB_LOGD("BSS 0x%x..0x%x\n", &_bss_start, &_bss_end);
	STUB_LOGD("cmd %d:%s\n", cmd, cmd_to_str(cmd));

	va_start(ap, cmd);
	if (cmd <= ESP_STUB_CMD_FLASH_MAX_ID) {
		ret = stub_flash_handler(cmd, ap);
	} else {
		switch (cmd) {
#if STUB_DEBUG
		case ESP_STUB_CMD_TEST:
			STUB_LOGD("TEST %d\n", cmd);
			break;
#endif
		default:
			ret = ESP_STUB_ERR_NOT_SUPPORTED;
		}
	}
	va_end(ap);

	STUB_LOGD("exit %d\n", ret);
	return ret;
}
