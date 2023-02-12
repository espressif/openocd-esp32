/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Espressif gpio driver based on imx_gpio.c                             *
 *   Copyright (C) 2022 Espressif Systems (Shanghai) Co. Ltd.              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"

/* esp-idf includes */
#include <driver/gpio.h>
#include <driver/dedic_gpio.h>
#include <hal/dedic_gpio_cpu_ll.h>

static dedic_gpio_bundle_handle_t gpio_in_bundle;
static dedic_gpio_bundle_handle_t gpio_out_bundle;

/* mask values depends on the location in the gpio bundle array */
#define GPIO_TDO_MASK       0x01	/* input */

/* outputs */
#define GPIO_TCK_MASK       0x01
#define GPIO_TDI_MASK       0x02
#define GPIO_TMS_MASK       0x04
#define GPIO_TRST_MASK      0x08
#define GPIO_SRST_MASK      0x10
#define GPIO_BLINK_MASK     0x20

/* GPIO setup functions */
static inline void gpio_mode_input_set(int g)
{
	gpio_set_direction(g, GPIO_MODE_INPUT);
}

static inline void gpio_mode_output_set(int g)
{
	gpio_set_direction(g, GPIO_MODE_OUTPUT);
}

static inline void gpio_set(int g)
{
	gpio_set_level(g, 1);
}

static inline void gpio_clear(int g)
{
	gpio_set_level(g, 0);
}

static bb_value_t esp32_gpio_read(void);
static int esp32_gpio_write(int tck, int tms, int tdi);
static int esp32_gpio_reset(int trst, int srst);
static int esp32_gpio_blink(int on);

static int esp32_gpio_init(void);
static int esp32_gpio_quit(void);

static struct bitbang_interface esp32_gpio_bitbang = {
	.read = esp32_gpio_read,
	.write = esp32_gpio_write,
	.swdio_read = NULL,
	.swdio_drive = NULL,
	.blink = NULL
};

/* GPIO default values for each pin */
static int tck_gpio = GPIO_NUM_NC;
static int tms_gpio = GPIO_NUM_NC;
static int tdi_gpio = GPIO_NUM_NC;
static int tdo_gpio = GPIO_NUM_NC;
static int trst_gpio = GPIO_NUM_NC;
static int srst_gpio = GPIO_NUM_NC;
static int blink_gpio = GPIO_NUM_NC;

static unsigned int jtag_delay = 0;

static bb_value_t esp32_gpio_read(void)
{
	/* we have only one input and it's mask value is 0x01. So we don't need to check the mask value. */
	return dedic_gpio_cpu_ll_read_in();
}

static int esp32_gpio_write(int tck, int tms, int tdi)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_TMS_MASK, tms ? GPIO_TMS_MASK : 0);
	dedic_gpio_cpu_ll_write_mask(GPIO_TDI_MASK, tdi ? GPIO_TDI_MASK : 0);
	dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, tck ? GPIO_TCK_MASK : 0);

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int esp32_gpio_blink(int on)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_BLINK_MASK, on ? GPIO_BLINK_MASK : 0);
	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int esp32_gpio_reset(int trst, int srst)
{
	if (trst_gpio != GPIO_NUM_NC)
		dedic_gpio_cpu_ll_write_mask(GPIO_TRST_MASK, trst ? GPIO_TRST_MASK : 0);

	if (srst_gpio != GPIO_NUM_NC)
		dedic_gpio_cpu_ll_write_mask(GPIO_SRST_MASK, srst ? GPIO_SRST_MASK : 0);

	return ERROR_OK;
}

int s_jtag_speed_khz;

static int esp32_gpio_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	s_jtag_speed_khz = khz;
	*jtag_speed = 0;
	return ERROR_OK;
}

static int esp32_gpio_speed_div(int speed, int *khz)
{
	*khz = s_jtag_speed_khz;
	return ERROR_OK;
}

static int esp32_gpio_speed(int speed)
{
	/* jtag_delay = speed; */
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
		"esp32_gpio GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
		tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "esp32_gpio GPIO config: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "esp32_gpio GPIO config: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "esp32_gpio GPIO config: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "esp32_gpio GPIO config: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "esp32_gpio GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "esp32_gpio GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_gpio_handle_jtag_gpionum_blink)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], blink_gpio);

	command_print(CMD, "esp32_gpio GPIO config: blink = %d", blink_gpio);
	return ERROR_OK;
}

static const struct command_registration esp32_gpio_command_handlers[] = {
	{
		.name = "esp32_gpio_jtag_nums",
		.handler = &esp32_gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "esp32_gpio_tck_num",
		.handler = &esp32_gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "esp32_gpio_tms_num",
		.handler = &esp32_gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "esp32_gpio_tdo_num",
		.handler = &esp32_gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "esp32_gpio_tdi_num",
		.handler = &esp32_gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "esp32_gpio_srst_num",
		.handler = &esp32_gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "esp32_gpio_trst_num",
		.handler = &esp32_gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "esp32_gpio_blink_num",
		.handler = &esp32_gpio_handle_jtag_gpionum_blink,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for blink.",
		.usage = "[blink]",
	},

	COMMAND_REGISTRATION_DONE
};

static const char *const esp32_gpio_transports[] = { "jtag", NULL };

static struct jtag_interface esp32_gpio_jtag_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver esp32_gpio_interface = {
	.name = "esp32_gpio",
	.transports = esp32_gpio_transports,
	/* .swd = NULL, */
	.speed = esp32_gpio_speed,
	.khz = esp32_gpio_khz,
	.speed_div = esp32_gpio_speed_div,
	.commands = esp32_gpio_command_handlers,
	.init = esp32_gpio_init,
	.quit = esp32_gpio_quit,
	.reset = esp32_gpio_reset,
	.jtag_ops = &esp32_gpio_jtag_interface,
};

static bool esp32_gpio_jtag_mode_possible(void)
{
	if (!GPIO_IS_VALID_OUTPUT_GPIO(tck_gpio))
		return 0;
	if (!GPIO_IS_VALID_OUTPUT_GPIO(tms_gpio))
		return 0;
	if (!GPIO_IS_VALID_GPIO(tdi_gpio))
		return 0;
	if (!GPIO_IS_VALID_OUTPUT_GPIO(tdo_gpio))
		return 0;
	return 1;
}

static int esp32_gpio_init(void)
{
	LOG_INFO("esp32_gpio GPIO JTAG bitbang driver");

	int khz = 5000;
	int jtag_speed = 0;

	esp32_gpio_khz(khz, &jtag_speed);

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	if (esp32_gpio_jtag_mode_possible()) {
		gpio_clear(tdi_gpio);
		gpio_clear(tck_gpio);
		gpio_set(tms_gpio);
		gpio_mode_input_set(tdo_gpio);
		gpio_mode_output_set(tdi_gpio);
		gpio_mode_output_set(tck_gpio);
		gpio_mode_output_set(tms_gpio);
	} else {
		LOG_ERROR("some JTAG pins are not set");
		return ERROR_FAIL;
	}

	int bundle_out_gpios[] = { tck_gpio, tdi_gpio, tms_gpio, 0, 0, 0 };
	int bundle_in_gpios[] = { tdo_gpio };

	if (trst_gpio != GPIO_NUM_NC) {
		gpio_set(trst_gpio);
		gpio_mode_output_set(trst_gpio);
		bundle_out_gpios[3] = trst_gpio;
	}
	if (srst_gpio != GPIO_NUM_NC) {
		gpio_set(srst_gpio);
		gpio_mode_output_set(srst_gpio);
		bundle_out_gpios[4] = srst_gpio;
	}
	if (blink_gpio != GPIO_NUM_NC) {
		gpio_clear(blink_gpio);
		gpio_mode_output_set(blink_gpio);
		bundle_out_gpios[5] = blink_gpio;
		esp32_gpio_bitbang.blink = esp32_gpio_blink;
	}

	dedic_gpio_bundle_config_t out_bundle_config = {
		.gpio_array = bundle_out_gpios,
		.array_size = ARRAY_SIZE(bundle_out_gpios),
		.flags = {
			.out_en = 1,
		},
	};

	dedic_gpio_bundle_config_t in_bundle_config = {
		.gpio_array = bundle_in_gpios,
		.array_size = ARRAY_SIZE(bundle_in_gpios),
		.flags = {
			.in_en = 1,
		},
	};

	dedic_gpio_new_bundle(&out_bundle_config, &gpio_out_bundle);
	dedic_gpio_new_bundle(&in_bundle_config, &gpio_in_bundle);

	bitbang_interface = &esp32_gpio_bitbang;

	return ERROR_OK;
}

static int esp32_gpio_quit(void)
{
	return ERROR_OK;
}
