// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   RiscV specific flasher stub functions                                 *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <stub_flasher.h>
#include <stub_flasher_chip.h>
#include <stub_flasher_int.h>
#include <stub_logger.h>

#define RISCV_EBREAK        0x9002

#define ESP_APPTRACE_RISCV_BLOCK_LEN_MSK                0x7FFFUL
#define ESP_APPTRACE_RISCV_BLOCK_LEN(_l_)               ((_l_) & ESP_APPTRACE_RISCV_BLOCK_LEN_MSK)
#define ESP_APPTRACE_RISCV_BLOCK_LEN_GET(_v_)           ((_v_) & ESP_APPTRACE_RISCV_BLOCK_LEN_MSK)
#define ESP_APPTRACE_RISCV_BLOCK_ID_MSK                 0x7FUL
#define ESP_APPTRACE_RISCV_BLOCK_ID(_id_)               (((_id_) & ESP_APPTRACE_RISCV_BLOCK_ID_MSK) << 15)
#define ESP_APPTRACE_RISCV_BLOCK_ID_GET(_v_)            (((_v_) >> 15) & ESP_APPTRACE_RISCV_BLOCK_ID_MSK)
#define ESP_APPTRACE_RISCV_HOST_DATA                    BIT(22)
#define ESP_APPTRACE_RISCV_HOST_CONNECT                 BIT(23)

/** RISCV memory host iface control block */
typedef struct {
	uint32_t ctrl;
	/* - Guard field. If this register is not zero then CPU is changing this struct and */
	/*   this guard field holds address of the instruction which application will execute when
	 * CPU finishes with those modifications. */
	uint32_t stat;
	esp_apptrace_mem_block_t *mem_blocks;
} esp_apptrace_riscv_ctrl_block_t;

static esp_apptrace_riscv_ctrl_block_t *s_apptrace_ctrl;

#if CONFIG_STUB_STACK_DATA_POOL_SIZE > 0
static uint8_t *s_stack_data_pool;
static size_t s_stack_data_pool_sz;

void stub_stack_data_pool_init(uint8_t *data, size_t sz)
{
	STUB_LOGD("stack data pool %lu bytes @ 0x%x\n", sz, data);
	s_stack_data_pool = data;
	s_stack_data_pool_sz = sz;
}

void esp_apptrace_get_up_buffers(esp_apptrace_mem_block_t mem_blocks_cfg[2])
{
	/* use whole stack data pool for apptrace up buffers */
	mem_blocks_cfg[0].start = s_stack_data_pool;
	mem_blocks_cfg[0].sz = s_stack_data_pool_sz / 2;
	mem_blocks_cfg[1].start = s_stack_data_pool + mem_blocks_cfg[0].sz;
	mem_blocks_cfg[1].sz = mem_blocks_cfg[0].sz;

	STUB_LOGD("Apptrace memory blocks: [0] %d bytes @ 0x%x, [1] %d bytes @ 0x%x\n",
		mem_blocks_cfg[0].sz, mem_blocks_cfg[0].start,
		mem_blocks_cfg[1].sz, mem_blocks_cfg[1].start);
}
#endif

/* override apptrace control block advertising func, IDF's implementation issues syscall */
int esp_apptrace_advertise_ctrl_block(void *ctrl_block_addr)
{
	STUB_LOGD("ctrl_block_addr %p\n", ctrl_block_addr);
	s_apptrace_ctrl = ctrl_block_addr;

	return ESP_STUB_ERR_OK;
}

int stub_apptrace_prepare(void)
{
	/* imply that host is auto-connected */
	s_apptrace_ctrl->ctrl |= ESP_APPTRACE_RISCV_HOST_CONNECT;

	return ESP_STUB_ERR_OK;
}

void vPortEnterCritical(void)
{
}

void vPortExitCritical(void)
{
}

uint32_t stub_flash_get_id(void)
{
	uint32_t ret;

	STUB_LOGD("flash %x, cs %x, bs %x, ss %x, ps %x, sm %x\n",
		rom_spiflash_legacy_data->chip.device_id,
		rom_spiflash_legacy_data->chip.chip_size,
		rom_spiflash_legacy_data->chip.block_size,
		rom_spiflash_legacy_data->chip.sector_size,
		rom_spiflash_legacy_data->chip.page_size,
		rom_spiflash_legacy_data->chip.status_mask);
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0);	/* clear register */
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_MEM_FLASH_RDID);
	while (READ_PERI_REG(PERIPHS_SPI_FLASH_CMD) != 0)
		;
	ret = READ_PERI_REG(PERIPHS_SPI_FLASH_C0) & 0xffffff;
	STUB_LOGD("Flash ID read %x\n", ret);
	return ret >> 16;
}

uint32_t stub_get_break_insn(uint8_t insn_sz)
{
	return RISCV_EBREAK;
}

uint8_t stub_get_insn_size(uint8_t *insn)
{
	/* we use 16bit `c.ebreak`. it works perfectly with either 32bit and 16bit code */
	return 2;
}

uint8_t stub_get_max_insn_size(void)
{
	return 2;
}
