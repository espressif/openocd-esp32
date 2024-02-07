/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_JTAG_DRIVERS_ESP_GPIO_H
#define OPENOCD_JTAG_DRIVERS_ESP_GPIO_H

#include <driver/gpio.h>
#include "hal/gpio_ll.h"

extern gpio_dev_t *const s_gpio_dev;
extern unsigned int s_gpio_delay;
extern int s_tck_gpio;
extern int s_tms_gpio;
extern int s_tdi_gpio;
extern int s_tdo_gpio;
extern int s_trst_gpio;
extern int s_srst_gpio;
extern int s_blink_gpio;
extern int s_swdio_gpio;
extern int s_swclk_gpio;

/* GPIO setup functions */
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

#if CONFIG_SOC_DEDICATED_GPIO_SUPPORTED

/* esp-idf includes */
#include <driver/dedic_gpio.h>
#include <hal/dedic_gpio_cpu_ll.h>

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

static inline int esp_gpio_swdio_read(void)
{
	return dedic_gpio_cpu_ll_read_in();
}

static inline int esp_gpio_swd_write(int swclk, int swdio)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_SWCLK_MASK, swclk ? GPIO_SWCLK_MASK : 0);
	dedic_gpio_cpu_ll_write_mask(GPIO_SWDIO_OUT_MASK, swdio ? GPIO_SWDIO_OUT_MASK : 0);

	for (unsigned int i = 0; i < s_gpio_delay; i++)
		asm volatile ("");

	return ESP_OK;
}

static inline int esp_gpio_swd_blink(int on)
{
	dedic_gpio_cpu_ll_write_mask(GPIO_SWD_BLINK_MASK, on ? GPIO_SWD_BLINK_MASK : 0);
	return ERROR_OK;
}

static inline int esp_gpio_jtag_read(void)
{
	/* we have only one input and it's mask value is 0x01. So we don't need to check the mask value. */
	return dedic_gpio_cpu_ll_read_in();
}

static inline int esp_gpio_jtag_write(int tck, int tms, int tdi)
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
static inline int esp_gpio_jtag_reset(int trst, int srst)
{
	if (s_trst_gpio != GPIO_NUM_NC)
		dedic_gpio_cpu_ll_write_mask(GPIO_TRST_MASK, trst ? GPIO_TRST_MASK : 0);

	if (s_srst_gpio != GPIO_NUM_NC)
		dedic_gpio_cpu_ll_write_mask(GPIO_SRST_MASK, srst ? GPIO_SRST_MASK : 0);

	return ERROR_OK;
}

#else

static inline int esp_gpio_swdio_read(void)
{
	return gpio_ll_get_level(s_gpio_dev, s_swdio_gpio);
}

static inline int esp_gpio_swd_write(int swclk, int swdio)
{
	gpio_ll_set_level(s_gpio_dev, s_swclk_gpio, swclk ? 1 : 0);
	gpio_ll_set_level(s_gpio_dev, s_swdio_gpio, swdio ? 1 : 0);

	for (unsigned int i = 0; i < s_gpio_delay; i++)
		asm volatile ("");

	return ESP_OK;
}

static inline int esp_gpio_swd_blink(int on)
{
	gpio_ll_set_level(s_gpio_dev, s_blink_gpio, on ? 1 : 0);
	return ERROR_OK;
}

static inline int esp_gpio_jtag_read(void)
{
	return gpio_ll_get_level(s_gpio_dev, s_tdo_gpio);
}

static inline int esp_gpio_jtag_write(int tck, int tms, int tdi)
{
	gpio_ll_set_level(s_gpio_dev, s_tms_gpio, tms ? 1 : 0);
	gpio_ll_set_level(s_gpio_dev, s_tdi_gpio, tdi ? 1 : 0);
	gpio_ll_set_level(s_gpio_dev, s_tck_gpio, tck ? 1 : 0);

	for (unsigned int i = 0; i < s_gpio_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

static int esp_gpio_jtag_blink(int on)
{
	gpio_ll_set_level(s_gpio_dev, s_blink_gpio, on ? 1 : 0);
	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static inline int esp_gpio_jtag_reset(int trst, int srst)
{
	if (s_trst_gpio != GPIO_NUM_NC)
		gpio_ll_set_level(s_gpio_dev, s_trst_gpio, trst ? 1 : 0);

	if (s_srst_gpio != GPIO_NUM_NC)
		gpio_ll_set_level(s_gpio_dev, s_srst_gpio, srst ? 1 : 0);

	return ERROR_OK;
}

#endif /* CONFIG_SOC_DEDICATED_GPIO_SUPPORTED */

#endif /* OPENOCD_JTAG_DRIVERS_ESP_GPIO_H */
