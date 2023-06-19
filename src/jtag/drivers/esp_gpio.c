/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Espressif gpio driver based on imx_gpio.c                             *
 *   Copyright (C) 2022 Espressif Systems (Shanghai) Co. Ltd.              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

/* esp-idf includes */
#include <driver/gpio.h>
#include <driver/dedic_gpio.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include "hal/gpio_ll.h"

static dedic_gpio_bundle_handle_t s_gpio_in_bundle;
static dedic_gpio_bundle_handle_t s_gpio_out_bundle;
static dedic_gpio_bundle_handle_t s_gpio_io_bundle;

/* mask values depends on the location in the gpio bundle array */
/* SWD io pin mask values */
#define GPIO_SWDIO_MASK			0x01    /* bundle_io_gpios[0] */ /* input/output */
#define GPIO_SWDIO_MAX_MASK		0x02	/* will be used as io array size */

/* SWD out pin mask values */
#define GPIO_SWCLK_MASK			0x01	/* bundle_out_gpios[0] */
#define GPIO_SWD_BLINK_MASK		0x02	/* bundle_out_gpios[1] */
#define GPIO_SWD_OUT_MAX_MASK	0x04	/* will be used as out array size */
#define GPIO_SWDIO_OUT_MASK		0x04	/* will not be in the out array, but it should follow the previous pin mask */

/* JTAG input pins mask values */
#define GPIO_TDO_MASK			0x01
#define GPIO_JTAG_IN_MAX_MASK	0x02

/* JTAG output pins mask values */
#define GPIO_TCK_MASK			0x01
#define GPIO_TDI_MASK			0x02
#define GPIO_TMS_MASK			0x04
#define GPIO_TRST_MASK			0x08
#define GPIO_SRST_MASK			0x10
#define GPIO_JTAG_BLINK_MASK	0x20
#define GPIO_JTAG_OUT_MAX_MASK	0x40

#define GET_IDX(mask) (__builtin_ctz(mask))

static uint32_t s_gpio_conf;
static gpio_dev_t *const s_gpio_dev = GPIO_LL_GET_HW(GPIO_PORT_0);

/* GPIO setup functions */
static inline void gpio_mode_input_set(int g)
{
	gpio_ll_output_disable(s_gpio_dev, g);
	gpio_ll_input_enable(s_gpio_dev, g);
}

static inline void gpio_mode_output_set(int g)
{
	gpio_ll_input_disable(s_gpio_dev, g);
	gpio_ll_output_enable(s_gpio_dev, g);
}

static inline void gpio_mode_input_output_set(int g)
{
	gpio_ll_input_enable(s_gpio_dev, g);
	gpio_ll_output_enable(s_gpio_dev, g);
}

static inline void gpio_set(int g)
{
	gpio_ll_set_level(s_gpio_dev, g, 1);
}

static inline void gpio_clear(int g)
{
	gpio_ll_set_level(s_gpio_dev, g, 0);
}

static bb_value_t esp_gpio_jtag_read(void);
static int esp_gpio_jtag_write(int tck, int tms, int tdi);
static int esp_gpio_jtag_reset(int trst, int srst);
static int esp_gpio_jtag_blink(int on);

static int esp_gpio_init(void);
static int esp_gpio_quit(void);

static void esp_gpio_swdio_drive(bool is_output);
static int esp_gpio_swdio_read(void);
static int esp_gpio_swd_write(int swclk, int swdio);
static int esp_gpio_swd_blink(int on);

static struct bitbang_interface esp_gpio_bitbang = {
	.read = esp_gpio_jtag_read,
	.write = esp_gpio_jtag_write,
	.swdio_read = esp_gpio_swdio_read,
	.swdio_drive = esp_gpio_swdio_drive,
	.swd_write = esp_gpio_swd_write,
	.blink = NULL
};

/* GPIO default values for each pin */
static int s_tck_gpio = GPIO_NUM_NC;
static int s_tms_gpio = GPIO_NUM_NC;
static int s_tdi_gpio = GPIO_NUM_NC;
static int s_tdo_gpio = GPIO_NUM_NC;
static int s_trst_gpio = GPIO_NUM_NC;
static int s_srst_gpio = GPIO_NUM_NC;
static int s_blink_gpio = GPIO_NUM_NC;
static int s_swdio_gpio = GPIO_NUM_NC;
static int s_swclk_gpio = GPIO_NUM_NC;

static unsigned int s_gpio_delay;

static void esp_gpio_swdio_drive(bool is_output)
{
	if (is_output) {
		REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (s_swdio_gpio * 4), s_gpio_conf);
		gpio_ll_output_enable(s_gpio_dev, s_swdio_gpio);
	} else {
		/* Disabling the output */
		gpio_ll_output_disable(s_gpio_dev, s_swdio_gpio);
	}
}

static int esp_gpio_swdio_read(void)
{
	return dedic_gpio_cpu_ll_read_in();
}

static int esp_gpio_swd_write(int swclk, int swdio)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_SWCLK_MASK, swclk ? GPIO_SWCLK_MASK : 0);
	dedic_gpio_cpu_ll_write_mask(GPIO_SWDIO_OUT_MASK, swdio ? GPIO_SWDIO_OUT_MASK : 0);

	for (unsigned int i = 0; i < s_gpio_delay; i++)
		asm volatile ("");

	return ESP_OK;
}

static int esp_gpio_swd_blink(int on)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_SWD_BLINK_MASK, on ? GPIO_SWD_BLINK_MASK : 0);
	return ERROR_OK;
}

static bb_value_t esp_gpio_jtag_read(void)
{
	/* we have only one input and it's mask value is 0x01. So we don't need to check the mask value. */
	return dedic_gpio_cpu_ll_read_in();
}

static int esp_gpio_jtag_write(int tck, int tms, int tdi)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_TMS_MASK, tms ? GPIO_TMS_MASK : 0);
	dedic_gpio_cpu_ll_write_mask(GPIO_TDI_MASK, tdi ? GPIO_TDI_MASK : 0);
	dedic_gpio_cpu_ll_write_mask(GPIO_TCK_MASK, tck ? GPIO_TCK_MASK : 0);

	for (unsigned int i = 0; i < s_gpio_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int esp_gpio_jtag_blink(int on)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_JTAG_BLINK_MASK, on ? GPIO_JTAG_BLINK_MASK : 0);
	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int esp_gpio_jtag_reset(int trst, int srst)
{
	if (s_trst_gpio != GPIO_NUM_NC)
		dedic_gpio_cpu_ll_write_mask(GPIO_TRST_MASK, trst ? GPIO_TRST_MASK : 0);

	if (s_srst_gpio != GPIO_NUM_NC)
		dedic_gpio_cpu_ll_write_mask(GPIO_SRST_MASK, srst ? GPIO_SRST_MASK : 0);

	return ERROR_OK;
}

int s_gpio_speed_khz;

static int esp_gpio_khz(int khz, int *speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	s_gpio_speed_khz = khz;
	*speed = 0;
	return ERROR_OK;
}

static int esp_gpio_speed_div(int speed, int *khz)
{
	*khz = s_gpio_speed_khz;
	return ERROR_OK;
}

static int esp_gpio_speed(int speed)
{
	s_gpio_delay = speed;
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], s_tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], s_tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], s_tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
		"esp_gpio GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
		s_tck_gpio, s_tms_gpio, s_tdi_gpio, s_tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_tck_gpio);

	command_print(CMD, "esp_gpio GPIO config: tck = %d", s_tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_tms_gpio);

	command_print(CMD, "esp_gpio GPIO config: tms = %d", s_tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_tdo_gpio);

	command_print(CMD, "esp_gpio GPIO config: tdo = %d", s_tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_tdi_gpio);

	command_print(CMD, "esp_gpio GPIO config: tdi = %d", s_tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_srst_gpio);

	command_print(CMD, "esp_gpio GPIO config: srst = %d", s_srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_trst_gpio);

	command_print(CMD, "esp_gpio GPIO config: trst = %d", s_trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_gpionum_blink)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_blink_gpio);

	command_print(CMD, "esp_gpio GPIO config: blink = %d", s_blink_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], s_swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"esp_gpio GPIO nums: swclk = %d, swdio = %d",
			s_swclk_gpio, s_swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_swclk_gpio);

	command_print(CMD, "esp_gpio num: swclk = %d", s_swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(esp_gpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], s_swdio_gpio);

	command_print(CMD, "esp_gpio num: swdio = %d", s_swdio_gpio);
	return ERROR_OK;
}

static const struct command_registration esp_gpio_command_handlers[] = {
	{
		.name = "esp_gpio_jtag_nums",
		.handler = &esp_gpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "esp_gpio_tck_num",
		.handler = &esp_gpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "esp_gpio_tms_num",
		.handler = &esp_gpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "esp_gpio_tdo_num",
		.handler = &esp_gpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "esp_gpio_tdi_num",
		.handler = &esp_gpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "esp_gpio_srst_num",
		.handler = &esp_gpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "esp_gpio_trst_num",
		.handler = &esp_gpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "esp_gpio_blink_num",
		.handler = &esp_gpio_handle_gpionum_blink,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for blink.",
		.usage = "[blink]",
	},
	{
		.name = "esp_gpio_swd_nums",
		.handler = &esp_gpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "esp_gpio_swclk_num",
		.handler = &esp_gpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "esp_gpio_swdio_num",
		.handler = &esp_gpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},

	COMMAND_REGISTRATION_DONE
};

static const char *const s_esp_gpio_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface s_esp_gpio_jtag_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver esp_gpio_adapter_driver = {
	.name = "esp_gpio",
	.transports = s_esp_gpio_transports,
	.speed = esp_gpio_speed,
	.khz = esp_gpio_khz,
	.speed_div = esp_gpio_speed_div,
	.commands = esp_gpio_command_handlers,
	.init = esp_gpio_init,
	.quit = esp_gpio_quit,
	.reset = esp_gpio_jtag_reset,
	.jtag_ops = &s_esp_gpio_jtag_interface,
	.swd_ops = &bitbang_swd,
};

static bool esp_gpio_jtag_mode_possible(void)
{
	if (!GPIO_IS_VALID_OUTPUT_GPIO(s_tck_gpio))
		return 0;
	if (!GPIO_IS_VALID_OUTPUT_GPIO(s_tms_gpio))
		return 0;
	if (!GPIO_IS_VALID_GPIO(s_tdi_gpio))
		return 0;
	if (!GPIO_IS_VALID_OUTPUT_GPIO(s_tdo_gpio))
		return 0;
	return 1;
}

static bool esp_gpio_swd_mode_possible(void)
{
	if (!GPIO_IS_VALID_GPIO(s_swclk_gpio))
		return 0;
	if (!GPIO_IS_VALID_GPIO(s_swdio_gpio))
		return 0;
	return 1;
}

static int esp_gpio_init(void)
{
	LOG_INFO("esp_gpio GPIO JTAG/SWD bitbang driver");

	int khz = 5000;
	int speed = 0;

	esp_gpio_khz(khz, &speed);

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 */
	if (transport_is_jtag()) {
		if (esp_gpio_jtag_mode_possible()) {
			gpio_clear(s_tdi_gpio);
			gpio_clear(s_tck_gpio);
			gpio_set(s_tms_gpio);
			gpio_mode_input_set(s_tdo_gpio);
			gpio_mode_output_set(s_tdi_gpio);
			gpio_mode_output_set(s_tck_gpio);
			gpio_mode_output_set(s_tms_gpio);
		} else {
			LOG_ERROR("some JTAG pins are not set");
			return ERROR_FAIL;
		}

		int bundle_out_gpios[GET_IDX(GPIO_JTAG_OUT_MAX_MASK)] = { 0 };
		int bundle_in_gpios[GET_IDX(GPIO_JTAG_IN_MAX_MASK)] = { 0 };

		bundle_out_gpios[GET_IDX(GPIO_TCK_MASK)] = s_tck_gpio;
		bundle_out_gpios[GET_IDX(GPIO_TDI_MASK)] = s_tdi_gpio;
		bundle_out_gpios[GET_IDX(GPIO_TMS_MASK)] = s_tms_gpio;

		bundle_in_gpios[GET_IDX(GPIO_TDO_MASK)] = s_tdo_gpio;

		if (s_trst_gpio != GPIO_NUM_NC) {
			gpio_set(s_trst_gpio);
			gpio_mode_output_set(s_trst_gpio);
			bundle_out_gpios[GET_IDX(GPIO_TRST_MASK)] = s_trst_gpio;
		}
		if (s_srst_gpio != GPIO_NUM_NC) {
			gpio_set(s_srst_gpio);
			gpio_mode_output_set(s_srst_gpio);
			bundle_out_gpios[GET_IDX(GPIO_SRST_MASK)] = s_srst_gpio;
		}

		if (s_blink_gpio != GPIO_NUM_NC) {
			gpio_clear(s_blink_gpio);
			gpio_mode_output_set(s_blink_gpio);
			bundle_out_gpios[GET_IDX(GPIO_JTAG_BLINK_MASK)] = s_blink_gpio;
			esp_gpio_bitbang.blink = esp_gpio_jtag_blink;
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

		dedic_gpio_new_bundle(&out_bundle_config, &s_gpio_out_bundle);
		dedic_gpio_new_bundle(&in_bundle_config, &s_gpio_in_bundle);
	}

	if (transport_is_swd()) {
		if (esp_gpio_swd_mode_possible()) {
		    gpio_reset_pin(s_swdio_gpio);
			gpio_reset_pin(s_swclk_gpio);

			gpio_mode_input_output_set(s_swdio_gpio);
			gpio_mode_output_set(s_swclk_gpio);
		    gpio_set_pull_mode(s_swdio_gpio, GPIO_PULLUP_ONLY);

			int bundle_out_gpios[GET_IDX(GPIO_SWD_OUT_MAX_MASK)] = { 0 };
			int bundle_io_gpios[GET_IDX(GPIO_SWDIO_MAX_MASK)] = { 0 };

			if (s_blink_gpio != GPIO_NUM_NC) {
				gpio_clear(s_blink_gpio);
				gpio_mode_output_set(s_blink_gpio);
				bundle_out_gpios[GET_IDX(GPIO_SWD_BLINK_MASK)] = s_blink_gpio;
				esp_gpio_bitbang.blink = esp_gpio_swd_blink;
			}

			bundle_io_gpios[GET_IDX(GPIO_SWDIO_MASK)] = s_swdio_gpio;
			dedic_gpio_bundle_config_t io_bundle_config = {
				.gpio_array = bundle_io_gpios,
				.array_size = ARRAY_SIZE(bundle_io_gpios),
				.flags = {
					.out_en = 1,
					.in_en = 1,
				},
			};

			bundle_out_gpios[GET_IDX(GPIO_SWCLK_MASK)] = s_swclk_gpio;
			dedic_gpio_bundle_config_t out_bundle_config = {
				.gpio_array = bundle_out_gpios,
				.array_size = ARRAY_SIZE(bundle_out_gpios),
				.flags = {
					.out_en = 1,
				},
			};

			dedic_gpio_new_bundle(&out_bundle_config, &s_gpio_out_bundle);
			dedic_gpio_new_bundle(&io_bundle_config, &s_gpio_io_bundle);
			s_gpio_conf = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + (s_swdio_gpio * 4));
		} else {
			LOG_ERROR("some SWD pins are not set");
			return ERROR_FAIL;
		}
	}

	bitbang_interface = &esp_gpio_bitbang;
	return ERROR_OK;
}

static int esp_gpio_quit(void)
{
	return ERROR_OK;
}
