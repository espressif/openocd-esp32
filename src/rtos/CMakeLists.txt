# SPDX-License-Identifier: GPL-2.0-or-later

add_library(rtos STATIC)

target_sources(rtos PRIVATE
	rtos.c
	rtos_standard_stackings.c
	rtos_ecos_stackings.c
	rtos_chibios_stackings.c
	rtos_embkernel_stackings.c
	rtos_mqx_stackings.c
	rtos_ucos_iii_stackings.c
	rtos_freertos_stackings.c
	rtos_nuttx_stackings.c
	rtos_riot_stackings.c
	freertos.c
	threadx.c
	ecos.c
	linux.c
	chibios.c
	chromium-ec.c
	embkernel.c
	mqx.c
	ucos_iii.c
	nuttx.c
	rtkernel.c
	hwthread.c
	zephyr.c
	riot.c
	rtos.h
	rtos_standard_stackings.h
	rtos_ecos_stackings.h
	rtos_freertos_stackings.h
	linux_header.h
	rtos_chibios_stackings.h
	rtos_embkernel_stackings.h
	rtos_mqx_stackings.h
	rtos_riot_stackings.h
	rtos_ucos_iii_stackings.h
	rtos_nuttx_stackings.h
)

