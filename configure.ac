# SPDX-License-Identifier: GPL-2.0-or-later

AC_PREREQ([2.69])
AC_INIT([openocd], [v0.12.0],
  [OpenOCD Mailing List <openocd-devel@lists.sourceforge.net>])
AC_CONFIG_SRCDIR([src/openocd.c])
AC_CONFIG_AUX_DIR([build-aux])

m4_include([config_subdir.m4])dnl

# check for makeinfo before calling AM_INIT_AUTOMAKE
AC_CHECK_PROG([MAKEINFO], [makeinfo], [makeinfo])
AS_IF([test "x$MAKEINFO" = "x"], [
  MAKEINFO='echo makeinfo missing; true'
  AC_MSG_WARN([Info documentation will not be built.])
])
AC_SUBST([MAKEINFO])

AM_INIT_AUTOMAKE([-Wall -Wno-portability dist-bzip2 dist-zip subdir-objects])

AC_CONFIG_HEADERS([config.h])

AC_LANG([C])
AC_PROG_CC
# autoconf 2.70 obsoletes AC_PROG_CC_C99 and includes it in AC_PROG_CC
m4_version_prereq([2.70],[],[AC_PROG_CC_C99])
AC_PROG_RANLIB

# If macro PKG_PROG_PKG_CONFIG is not available, Autoconf generates a misleading error message,
# so check for existence first, and otherwise provide helpful advice.
m4_ifndef([PKG_PROG_PKG_CONFIG], [m4_fatal(m4_normalize([
  Macro PKG_PROG_PKG_CONFIG is not available.
  It is usually defined in file pkg.m4 provided by package pkg-config.]))])
PKG_PROG_PKG_CONFIG([0.23])

dnl disable checks for C++, Fortran and GNU Java Compiler
m4_defun([_LT_AC_LANG_CXX_CONFIG], [:])
m4_defun([_LT_AC_LANG_F77_CONFIG], [:])
m4_defun([_LT_AC_LANG_GCJ_CONFIG], [:])
AC_DISABLE_SHARED
LT_INIT
AC_SUBST([LIBTOOL_DEPS])

dnl configure checks required for Jim files (these are obsolete w/ C99)
AC_C_CONST
AC_TYPE_LONG_LONG_INT

AC_SEARCH_LIBS([ioperm], [ioperm])
AC_SEARCH_LIBS([dlopen], [dl])
AC_SEARCH_LIBS([openpty], [util])

AC_CHECK_HEADERS([sys/socket.h])
AC_CHECK_HEADERS([elf.h])

AC_CHECK_TYPE([Elf64_Ehdr],
	AC_DEFINE([HAVE_ELF64], [1], [Define to 1 if the system has the type 'Elf64_Ehdr'.]),
	[], [[#include <elf.h>]])

AC_MSG_CHECKING([for glibc])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[#include <features.h>]], [[int v = __GLIBC__;return 0;]])],
	[have_glibc=yes], [have_glibc=no])
AC_MSG_RESULT($have_glibc)

AC_CHECK_HEADERS([dirent.h])
AC_CHECK_HEADERS([fcntl.h])
AC_CHECK_HEADERS([linux/pci.h])
AC_CHECK_HEADERS([linux/spi/spidev.h])
AC_CHECK_HEADERS([malloc.h])
AC_CHECK_HEADERS([netdb.h])
AC_CHECK_HEADERS([poll.h])
AC_CHECK_HEADERS([strings.h])
AC_CHECK_HEADERS([sys/ioctl.h])
AC_CHECK_HEADERS([sys/param.h])
AC_CHECK_HEADERS([sys/select.h])
AC_CHECK_HEADERS([sys/stat.h])
AC_CHECK_HEADERS([sys/sysctl.h])
AC_CHECK_HEADERS([sys/time.h])
AC_CHECK_HEADERS([sys/types.h])
AC_CHECK_HEADERS([unistd.h])
AC_CHECK_HEADERS([arpa/inet.h netinet/in.h netinet/tcp.h], [], [], [dnl
#include <stdio.h>
#ifdef STDC_HEADERS
# include <stdlib.h>
# include <stddef.h>
#else
# ifdef HAVE_STDLIB_H
#  include <stdlib.h>
# endif
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])

AC_HEADER_ASSERT
AC_HEADER_STDBOOL

AC_C_BIGENDIAN

AC_CHECK_FUNCS([strndup])
AC_CHECK_FUNCS([strnlen])
AC_CHECK_FUNCS([gettimeofday])
AC_CHECK_FUNCS([usleep])
AC_CHECK_FUNCS([realpath])

# guess-rev.sh only exists in the repository, not in the released archives
AC_MSG_CHECKING([whether to build a release])
AS_IF([test -x "$srcdir/guess-rev.sh"], [
  build_release=no
], [
  build_release=yes
])
AC_MSG_RESULT([$build_release])

# Adapter drivers
# 1st column -- Basename for the configure option generated with AC_ARG_ENABLE.
#               For example, "buspirate" generates options "--enable-buspirate[=yes/no]"
#               and "--disable-buspirate".
# 2nd column -- Description for the configure option. For example, "Bus Pirate"
#               generates "Enable building support for the Bus Pirate (default is auto)".
# 3rd column -- Basename for the config.h and Automake symbols.
#               For example, basename "BUS_PIRATE" generates "BUILD_BUS_PIRATE" with AC_DEFINE
#               for config.h and "BUS_PIRATE" with AM_CONDITIONAL for Automake.
m4_define([ADAPTER_ARG], [m4_argn([1], $1)])
m4_define([ADAPTER_DESC], [m4_argn([2], $1)])
m4_define([ADAPTER_SYM], [m4_argn([3], $1)])
# AC_ARG_ENABLE uses prefix "enable_" to name the corresponding option variable.
m4_define([ADAPTER_VAR], [enable_[]ADAPTER_ARG($1)])
m4_define([ADAPTER_OPT], [m4_translit(ADAPTER_ARG($1), [_], [-])])

m4_define([USB1_ADAPTERS],
	[[[ftdi], [MPSSE mode of FTDI based devices], [FTDI]],
	[[stlink], [ST-Link Programmer], [HLADAPTER_STLINK]],
	[[ti_icdi], [TI ICDI JTAG Programmer], [HLADAPTER_ICDI]],
	[[ulink], [Keil ULINK JTAG Programmer], [ULINK]],
	[[angie], [ANGIE Adapter], [ANGIE]],
	[[usb_blaster_2], [Altera USB-Blaster II Compatible], [USB_BLASTER_2]],
	[[ft232r], [Bitbang mode of FT232R based devices], [FT232R]],
	[[vsllink], [Versaloon-Link JTAG Programmer], [VSLLINK]],
	[[xds110], [TI XDS110 Debug Probe], [XDS110]],
	[[osbdm], [OSBDM (JTAG only) Programmer], [OSBDM]],
	[[opendous], [eStick/opendous JTAG Programmer], [OPENDOUS]],
	[[armjtagew], [Olimex ARM-JTAG-EW Programmer], [ARMJTAGEW]],
	[[rlink], [Raisonance RLink JTAG Programmer], [RLINK]],
	[[usbprog], [USBProg JTAG Programmer], [USBPROG]],
	[[esp_usb_jtag], [Espressif JTAG Programmer], [ESP_USB_JTAG]],
	[[cmsis_dap_v2], [CMSIS-DAP v2 compliant dongle (USB bulk)], [CMSIS_DAP_USB]]])

# Please keep cmsis_dap_v2 the last in USB1_ADAPTERS
# and cmsis_dap the first in HIDAPI_ADAPTERS

m4_define([HIDAPI_ADAPTERS],
	[[[cmsis_dap], [CMSIS-DAP v1 compliant dongle (HID)], [CMSIS_DAP_HID]],
	[[nulink], [Nu-Link Programmer], [HLADAPTER_NULINK]]])

m4_define([HIDAPI_USB1_ADAPTERS],
	[[[kitprog], [Cypress KitProg Programmer], [KITPROG]]])

m4_define([LIBFTDI_ADAPTERS],
	[[[usb_blaster], [Altera USB-Blaster Compatible], [USB_BLASTER]],
	[[presto], [ASIX Presto Adapter], [PRESTO]]])

m4_define([LIBFTDI_USB1_ADAPTERS],
	[[[openjtag], [OpenJTAG Adapter], [OPENJTAG]]])

m4_define([LIBGPIOD_ADAPTERS],
	[[[linuxgpiod], [Linux GPIO bitbang through libgpiod], [LINUXGPIOD]]])

m4_define([DMEM_ADAPTER],
	[[[dmem], [CoreSight Direct Memory], [DMEM]]])

m4_define([SYSFSGPIO_ADAPTER],
	[[[sysfsgpio], [Linux GPIO bitbang through sysfs], [SYSFSGPIO]]])

m4_define([REMOTE_BITBANG_ADAPTER],
	[[[remote_bitbang], [Remote Bitbang driver], [REMOTE_BITBANG]]])

m4_define([LIBJAYLINK_ADAPTERS],
	[[[jlink], [SEGGER J-Link Programmer], [JLINK]]])

m4_define([PCIE_ADAPTERS],
	[[[xlnx_pcie_xvc], [Xilinx XVC/PCIe], [XLNX_PCIE_XVC]]])

m4_define([SERIAL_PORT_ADAPTERS],
	[[[buspirate], [Bus Pirate], [BUS_PIRATE]]])

m4_define([LINUXSPIDEV_ADAPTER],
	[[[linuxspidev], [Linux spidev driver], [LINUXSPIDEV]]])
m4_define([VDEBUG_ADAPTER],
	[[[vdebug], [Cadence Virtual Debug Interface], [VDEBUG]]])

m4_define([JTAG_DPI_ADAPTER],
	[[[jtag_dpi], [JTAG DPI Adapter], [JTAG_DPI]]])

m4_define([JTAG_VPI_ADAPTER],
	[[[jtag_vpi], [JTAG VPI Adapter], [JTAG_VPI]]])

m4_define([RSHIM_ADAPTER],
	[[[rshim], [BlueField SoC via rshim], [RSHIM]]])

m4_define([AMTJTAGACCEL_ADAPTER],
	[[[amtjtagaccel], [Amontec JTAG-Accelerator driver], [AMTJTAGACCEL]]])

m4_define([HOST_ARM_BITBANG_ADAPTERS],
	[[[ep93xx], [Bitbanging on EP93xx-based SBCs], [EP93XX]],
	[[at91rm9200], [Bitbanging on AT91RM9200-based SBCs], [AT91RM9200]]])

m4_define([HOST_ARM_OR_AARCH64_BITBANG_ADAPTERS],
	[[[bcm2835gpio], [Bitbanging on BCM2835 (as found in Raspberry Pi)], [BCM2835GPIO]],
	[[imx_gpio], [Bitbanging on NXP IMX processors], [IMX_GPIO]],
	[[am335xgpio], [Bitbanging on AM335x (as found in Beaglebones)], [AM335XGPIO]]])

m4_define([DUMMY_ADAPTER],
	[[[dummy], [Dummy Adapter], [DUMMY]]])

m4_define([OPTIONAL_LIBRARIES],
	[[[capstone], [Use Capstone disassembly framework], []]])

m4_define([COVERAGE],
	[[[gcov], [Collect coverage using gcov], []]])

AC_ARG_ENABLE([doxygen-html],
  AS_HELP_STRING([--disable-doxygen-html],
    [Disable building Doxygen manual as HTML.]),
  [doxygen_as_html=$enableval], [doxygen_as_html=yes])
AC_SUBST([doxygen_as_html])
AC_MSG_CHECKING([whether to build Doxygen as HTML])
AC_MSG_RESULT([$doxygen_as_html])

AC_ARG_ENABLE([doxygen-pdf],
  AS_HELP_STRING([--enable-doxygen-pdf],
    [Enable building Doxygen manual as PDF.]),
  [doxygen_as_pdf=$enableval], [doxygen_as_pdf=no])
AC_SUBST([doxygen_as_pdf])
AC_MSG_CHECKING([whether to build Doxygen as PDF])
AC_MSG_RESULT([$doxygen_as_pdf])

AC_ARG_ENABLE([gccwarnings],
  AS_HELP_STRING([--disable-gccwarnings], [Disable compiler warnings]),
  [gcc_warnings=$enableval], [gcc_warnings=yes])

AC_ARG_ENABLE([wextra],
  AS_HELP_STRING([--disable-wextra], [Disable extra compiler warnings]),
  [gcc_wextra=$enableval], [gcc_wextra=$gcc_warnings])

AC_ARG_ENABLE([werror],
  AS_HELP_STRING([--disable-werror], [Do not treat warnings as errors]),
  [gcc_werror=$enableval], [gcc_werror=$gcc_warnings])

AC_ARG_ENABLE([gcov],
  AS_HELP_STRING([--enable-gcov], [Enable runtime coverage collection via gcov]),
  [enable_gcov=$enableval], [enable_gcov=no])

AS_IF([test "x$enable_gcov" = "xyes"], [
  AC_DEFINE([USE_GCOV], [1], [1 to enable coverage collection using gcov.])
  dnl When collecting coverage, disable optimizations.
  dnl This overrides the "-O2" that autoconf uses by default:
  CFLAGS+=" -O0"
], [
  AC_DEFINE([USE_GCOV], [0], [0 to leave coverage collection disabled.])
])

# set default verbose options, overridden by following options
debug_usb_io=no
debug_usb_comms=no

AC_ARG_ENABLE([verbose],
  AS_HELP_STRING([--enable-verbose],
      [Enable verbose JTAG I/O messages (for debugging).]),
  [
  debug_usb_io=$enableval
  debug_usb_comms=$enableval
  ], [])

AC_ARG_ENABLE([verbose_usb_io],
  AS_HELP_STRING([--enable-verbose-usb-io],
      [Enable verbose USB I/O messages (for debugging)]),
  [debug_usb_io=$enableval], [])

AC_ARG_ENABLE([verbose_usb_comms],
  AS_HELP_STRING([--enable-verbose-usb-comms],
      [Enable verbose USB communication messages (for debugging)]),
  [debug_usb_comms=$enableval], [])

AC_MSG_CHECKING([whether to enable verbose USB I/O messages]);
AC_MSG_RESULT([$debug_usb_io])
AS_IF([test "x$debug_usb_io" = "xyes"], [
  AC_DEFINE([_DEBUG_USB_IO_],[1], [Print verbose USB I/O messages])
])

AC_MSG_CHECKING([whether to enable verbose USB communication messages]);
AC_MSG_RESULT([$debug_usb_comms])
AS_IF([test "x$debug_usb_comms" = "xyes"], [
  AC_DEFINE([_DEBUG_USB_COMMS_],[1], [Print verbose USB communication messages])
])

debug_malloc=no
AC_ARG_ENABLE([malloc_logging],
  AS_HELP_STRING([--enable-malloc-logging],
      [Include free space in logging messages (requires malloc.h).]),
  [debug_malloc=$enableval], [])

AC_MSG_CHECKING([whether to enable malloc free space logging]);
AC_MSG_RESULT([$debug_malloc])
AS_IF([test "x$debug_malloc" = "xyes" -a "x$have_glibc" = "xyes"], [
  AC_DEFINE([_DEBUG_FREE_SPACE_],[1], [Include malloc free space in logging])
])

m4_define([AC_ARG_ADAPTERS], [
  m4_foreach([adapterTuple], [$1],
	[AC_ARG_ENABLE(ADAPTER_OPT([adapterTuple]),
		AS_HELP_STRING([--enable-ADAPTER_OPT([adapterTuple])[[[=yes/no/auto]]]],
			[Enable building support for the ]ADAPTER_DESC([adapterTuple])[ (default is $2)]),
		[case "${enableval}" in
			yes|no|auto) ;;
			*) AC_MSG_ERROR([Option --enable-ADAPTER_OPT([adapterTuple]) has invalid value "${enableval}".]) ;;
			esac],
		[ADAPTER_VAR([adapterTuple])=$2])
  ])
])

AC_ARG_ADAPTERS([
  USB1_ADAPTERS,
  HIDAPI_ADAPTERS,
  HIDAPI_USB1_ADAPTERS,
  LIBFTDI_ADAPTERS,
  LIBFTDI_USB1_ADAPTERS,
  LIBGPIOD_ADAPTERS,
  DMEM_ADAPTER,
  SYSFSGPIO_ADAPTER,
  REMOTE_BITBANG_ADAPTER,
  LINUXSPIDEV_ADAPTER,
  SERIAL_PORT_ADAPTERS,
  DUMMY_ADAPTER,
  VDEBUG_ADAPTER,
  JTAG_DPI_ADAPTER,
  JTAG_VPI_ADAPTER,
  RSHIM_ADAPTER,
  PCIE_ADAPTERS,
  LIBJAYLINK_ADAPTERS
  ],[auto])

AC_ARG_ADAPTERS([
  AMTJTAGACCEL_ADAPTER
  ],[no])

# The following adapters use bitbanging and can actually be built on all architectures,
# which is useful to verify that they still build fine.
# We could enable them automatically only on the architectures where they actually occur:
# HOST_ARM_BITBANG_ADAPTERS:            when ${host_cpu} matches arm*
# HOST_ARM_OR_AARCH64_BITBANG_ADAPTERS: when ${host_cpu} matches arm*|aarch64
# However, conditionally changing the meaning of 'auto' requires
# a more flexible logic around.
AC_ARG_ADAPTERS([HOST_ARM_BITBANG_ADAPTERS],[no])
AC_ARG_ADAPTERS([HOST_ARM_OR_AARCH64_BITBANG_ADAPTERS],[no])

AC_ARG_ENABLE([parport],
  AS_HELP_STRING([--enable-parport], [Enable building the pc parallel port driver]),
  [build_parport=$enableval], [build_parport=no])

AC_ARG_ENABLE([parport_ppdev],
  AS_HELP_STRING([--disable-parport-ppdev],
      [Disable use of ppdev (/dev/parportN) for parport (for x86 only)]),
    [parport_use_ppdev=$enableval], [parport_use_ppdev=yes])

AC_ARG_ENABLE([parport_giveio],
    AS_HELP_STRING([--enable-parport-giveio],
      [Enable use of giveio for parport (for CygWin only)]),
    [parport_use_giveio=$enableval], [parport_use_giveio=])

AC_ARG_ENABLE([gw16012],
  AS_HELP_STRING([--enable-gw16012], [Enable building support for the Gateworks GW16012 JTAG Programmer]),
  [build_gw16012=$enableval], [build_gw16012=no])

can_build_rshim=no

AS_CASE([$host_os],
  [linux*], [
    is_linux=yes
    can_build_rshim=yes
  ],
  [
    AS_CASE([$host_os], [freebsd*], [
      can_build_rshim=yes
    ],
    [
      AS_IF([test "x$enable_rshim" = "xyes"], [
        AC_MSG_ERROR([rshim is only available on linux or freebsd])
      ])
    ])

    AS_IF([test "x$build_dmem" = "xyes"], [
      AC_MSG_ERROR([dmem is only available on linux])
    ])
])

AC_ARG_ENABLE([internal-jimtcl],
  AS_HELP_STRING([--enable-internal-jimtcl], [Enable building internal jimtcl (deprecated)]),
  [use_internal_jimtcl=$enableval], [use_internal_jimtcl=no])

AC_ARG_ENABLE([jimtcl-maintainer],
  AS_HELP_STRING([--enable-jimtcl-maintainer], [Enable maintainer mode when building internal jimtcl]),
  [use_internal_jimtcl_maintainer=$enableval], [use_internal_jimtcl_maintainer=no])

AC_ARG_ENABLE([internal-libjaylink],
  AS_HELP_STRING([--enable-internal-libjaylink],
  [Enable building internal libjaylink]),
  [use_internal_libjaylink=$enableval], [use_internal_libjaylink=no])

AC_ARG_ENABLE([esp-remote],
  AS_HELP_STRING([--disable-esp-remote],
  [Disable building support for the ESP remote protocol over TCP or USB]),
  [build_esp_remote=$enableval], [build_esp_remote=yes])

AC_ARG_ENABLE([esp-compression],
  AS_HELP_STRING([--disable-esp-compression],
  [Disable compression support for the ESP devices]),
  [build_esp_compression=$enableval], [build_esp_compression=yes])

AS_CASE(["${host_cpu}"],
  [i?86|x86*], [],
  [
    AS_IF([test "x$parport_use_ppdev" = "xno"], [
      AC_MSG_WARN([--disable-parport-ppdev is not supported by the host CPU])
    ])
    parport_use_ppdev=yes
])

can_build_buspirate=yes

AS_CASE([$host_os],
  [cygwin*], [
    is_win32=yes
    parport_use_ppdev=no

    AC_COMPILE_IFELSE([AC_LANG_PROGRAM([], [[return __MINGW32__;]])],
      [is_mingw=yes],[is_mingw=no])
    AS_IF([test "x$is_mingw" = "xyes"], [
      AS_IF([test "x$parport_use_giveio" = "xno"], [
        AC_MSG_WARN([--disable-parport-giveio is not supported by MinGW32 hosts])
      ])
      parport_use_giveio=yes
      is_cygwin=no
    ], [
      is_cygwin=yes
      # sys/io.h needed under cygwin for parport access
      AS_IF([test "x$build_parport" = "xyes"], [
        AC_CHECK_HEADERS([sys/io.h],[],AC_MSG_ERROR([Please install the cygwin ioperm package]))
      ])
    ])
  ],
  [mingw* | msys*], [
    is_mingw=yes
    is_win32=yes
    parport_use_ppdev=no

    AS_IF([test "x$parport_use_giveio" = "xno"], [
      AC_MSG_WARN([--disable-parport-giveio is not supported by MinGW32 hosts])
    ])
    parport_use_giveio=yes

    AS_IF([test "x$ADAPTER_VAR([buspirate])" = "xyes"], [
      AC_MSG_ERROR([The Bus Pirate adapter is currently not supported by MinGW32 hosts.])
    ])

    # In case enable_buspirate=auto, make sure it will not be built.
    can_build_buspirate=no

    AC_SUBST([HOST_CPPFLAGS], ["-D__USE_MINGW_ANSI_STDIO -DFD_SETSIZE=128"])
  ],
  [darwin*], [
    is_darwin=yes

    AS_IF([test "x$parport_use_giveio" = "xyes"], [
      AC_MSG_WARN([--enable-parport-giveio cannot be used by Darwin hosts])
    ])
    parport_use_giveio=no
  ],
  [
    AS_IF([test "x$parport_use_giveio" = "xyes"], [
      AC_MSG_WARN([--enable-parport-giveio cannot be used by ]$host[ hosts])
    ])
    parport_use_giveio=no
])

AS_IF([test "x$is_cygwin" = "xyes"], [
    AC_DEFINE([IS_CYGWIN], [1], [1 if building for Cygwin.])
], [
    AC_DEFINE([IS_CYGWIN], [0], [0 if not building for Cygwin.])
])

AS_IF([test "x$is_mingw" = "xyes"], [
    AC_DEFINE([IS_MINGW], [1], [1 if building for Mingw.])
], [
    AC_DEFINE([IS_MINGW], [0], [0 if not building for Mingw.])
])

AS_IF([test "x$is_win32" = "xyes"], [
    AC_DEFINE([IS_WIN32], [1], [1 if building for Win32.])
], [
    AC_DEFINE([IS_WIN32], [0], [0 if not building for Win32.])
])

AS_IF([test "x$is_darwin" = "xyes"], [
    AC_DEFINE([IS_DARWIN], [1], [1 if building for Darwin.])
], [
    AC_DEFINE([IS_DARWIN], [0], [0 if not building for Darwin.])
])

AC_DEFINE([IS_ESPIDF], [0], [ESP-IDF platform only supported with CMake build system])
AC_DEFINE([BUILD_ESP_GPIO], [0], [ESP-IDF platform only supported with CMake build system])

AS_IF([test "x$build_parport" = "xyes"], [
  build_bitbang=yes
  AC_DEFINE([BUILD_PARPORT], [1], [1 if you want parport.])
], [
  AC_DEFINE([BUILD_PARPORT], [0], [0 if you don't want parport.])
])

AS_IF([test "x$ADAPTER_VAR([dummy])" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$parport_use_ppdev" = "xyes"], [
  AC_DEFINE([PARPORT_USE_PPDEV], [1], [1 if you want parport to use ppdev.])
], [
  AC_DEFINE([PARPORT_USE_PPDEV], [0], [0 if you don't want parport to use ppdev.])
])

AS_IF([test "x$parport_use_giveio" = "xyes"], [
  AC_DEFINE([PARPORT_USE_GIVEIO], [1], [1 if you want parport to use giveio.])
], [
  AC_DEFINE([PARPORT_USE_GIVEIO], [0], [0 if you don't want parport to use giveio.])
])

AS_IF([test "x$build_gw16012" = "xyes"], [
  AC_DEFINE([BUILD_GW16012], [1], [1 if you want the Gateworks GW16012 driver.])
], [
  AC_DEFINE([BUILD_GW16012], [0], [0 if you don't want the Gateworks GW16012 driver.])
])

AS_IF([test "x$use_internal_jimtcl" = "xyes"], [
  AS_IF([test -f "$srcdir/jimtcl/configure"], [
    AS_IF([test "x$use_internal_jimtcl_maintainer" = "xyes"], [
      jimtcl_config_options="--disable-install-jim --with-ext=json --minimal --disable-ssl --maintainer"
    ], [
      jimtcl_config_options="--disable-install-jim --with-ext=json --minimal --disable-ssl"
    ])
    AX_CONFIG_SUBDIR_OPTION([jimtcl], [$jimtcl_config_options])
  ], [
    AC_MSG_ERROR([jimtcl not found, run git submodule init and git submodule update.])
  ])
], [
  PKG_CHECK_MODULES([JIMTCL], [jimtcl >= 0.79], [
      have_jimtcl_pkg_config=yes
    ], [
      have_jimtcl_pkg_config=no
      AC_CHECK_HEADER([jim.h], [], [
        AC_MSG_ERROR([jimtcl is required but not found via pkg-config and system includes])
      ])
  ])
])

AS_IF([test "x$build_esp_remote" = "xyes"], [
  AC_DEFINE([BUILD_ESP_REMOTE], [1], [1 if you want ESP remote protocol support.])
], [
  AC_DEFINE([BUILD_ESP_REMOTE], [0], [0 if you don't want ESP remote protocol support.])
])

AS_IF([test "x$build_esp_compression" = "xyes"], [
  AC_DEFINE([BUILD_ESP_COMPRESSION], [1], [1 if you want ESP compression support.])
  AC_SEARCH_LIBS([deflate], [z], [], [
    AC_MSG_ERROR([libz library was not found])])
  AC_CHECK_HEADERS([zlib.h])
], [
  AC_DEFINE([BUILD_ESP_COMPRESSION], [0], [0 if you don't want ESP compression support.])
])

PKG_CHECK_MODULES([LIBUSB1], [libusb-1.0], [
	use_libusb1=yes
	AC_DEFINE([HAVE_LIBUSB1], [1], [Define if you have libusb-1.x])
	LIBUSB1_CFLAGS=`echo $LIBUSB1_CFLAGS | sed 's/-I/-isystem /'`
	AC_MSG_NOTICE([libusb-1.0 header bug workaround: LIBUSB1_CFLAGS changed to "$LIBUSB1_CFLAGS"])
	PKG_CHECK_EXISTS([libusb-1.0 >= 1.0.16],
		[AC_DEFINE([HAVE_LIBUSB_GET_PORT_NUMBERS], [1], [Define if your libusb has libusb_get_port_numbers()])])
  ], [
	use_libusb1=no
	AC_MSG_WARN([libusb-1.x not found, trying legacy libusb-0.1 as a fallback; consider installing libusb-1.x instead])
])

AC_ARG_WITH([capstone],
		AS_HELP_STRING([--with-capstone], [Use Capstone disassembly library (default=auto)])
	, [
		enable_capstone=$withval
	], [
		enable_capstone=auto
])

AS_IF([test "x$enable_capstone" != xno], [
	PKG_CHECK_MODULES([CAPSTONE], [capstone], [
		AC_DEFINE([HAVE_CAPSTONE], [1], [1 if you have Capstone disassembly framework.])
	], [
		if test "x$enable_capstone" != xauto; then
			AC_MSG_ERROR([--with-capstone was given, but test for Capstone failed])
		fi
		enable_capstone=no
	])
])

AS_IF([test "x$enable_capstone" == xno], [
	AC_DEFINE([HAVE_CAPSTONE], [0], [0 if you don't have Capstone disassembly framework.])
])

for hidapi_lib in hidapi hidapi-hidraw hidapi-libusb; do
	PKG_CHECK_MODULES([HIDAPI],[$hidapi_lib],[
		use_hidapi=yes
		break
	],[
		use_hidapi=no
	])
done

PKG_CHECK_MODULES([LIBFTDI], [libftdi1], [
	use_libftdi=yes
	PKG_CHECK_EXISTS([libftdi1 >= 1.5],
		[AC_DEFINE([HAVE_LIBFTDI_TCIOFLUSH], [1], [Define if your libftdi has ftdi_tcioflush()])])
  ], [
	PKG_CHECK_MODULES([LIBFTDI], [libftdi], [use_libftdi=yes], [use_libftdi=no])
])

PKG_CHECK_MODULES([LIBGPIOD], [libgpiod < 2.0], [
	use_libgpiod=yes
	PKG_CHECK_EXISTS([libgpiod >= 1.5],
		[AC_DEFINE([HAVE_LIBGPIOD1_FLAGS_BIAS], [1], [define if libgpiod v1 has line request flags bias])])
], [use_libgpiod=no])

PKG_CHECK_MODULES([LIBJAYLINK], [libjaylink >= 0.2],
	[use_libjaylink=yes], [use_libjaylink=no])

# Arg $1: An array of adapter triplets, used to derive option and variable names for each adapter.
# Arg $2: Whether the adapters can be enabled, for example, because
#         their prerequisites are installed in the system.
# Arg $3: What prerequisites are missing, to be shown in an error message
#         if an adapter was requested but cannot be enabled.
m4_define([PROCESS_ADAPTERS], [
  m4_foreach([adapterTuple], [$1], [
	AS_IF([test $2], [
		AS_IF([test "x$ADAPTER_VAR([adapterTuple])" != "xno"], [
			AC_DEFINE([BUILD_]ADAPTER_SYM([adapterTuple]), [1],
			          [1 if you want the ]ADAPTER_DESC([adapterTuple]).)
		], [
			AC_DEFINE([BUILD_]ADAPTER_SYM([adapterTuple]), [0],
			          [0 if you do not want the ]ADAPTER_DESC([adapterTuple]).)
		])
	], [
		AS_IF([test "x$ADAPTER_VAR([adapterTuple])" = "xyes"], [
			AC_MSG_ERROR([$3 is required for [adapterTuple] "ADAPTER_DESC([adapterTuple])".])
		])
		ADAPTER_VAR([adapterTuple])=no
		AC_DEFINE([BUILD_]ADAPTER_SYM([adapterTuple]), [0], [0 if you do not want the ]ADAPTER_DESC([adapterTuple]).)
	])
	AM_CONDITIONAL(ADAPTER_SYM([adapterTuple]), [test "x$ADAPTER_VAR([adapterTuple])" != "xno"])
  ])
])

PROCESS_ADAPTERS([USB1_ADAPTERS], ["x$use_libusb1" = "xyes"], [libusb-1.x])
PROCESS_ADAPTERS([HIDAPI_ADAPTERS], ["x$use_hidapi" = "xyes"], [hidapi])
PROCESS_ADAPTERS([HIDAPI_USB1_ADAPTERS], ["x$use_hidapi" = "xyes" -a "x$use_libusb1" = "xyes"], [hidapi and libusb-1.x])
PROCESS_ADAPTERS([LIBFTDI_ADAPTERS], ["x$use_libftdi" = "xyes"], [libftdi])
PROCESS_ADAPTERS([LIBFTDI_USB1_ADAPTERS], ["x$use_libftdi" = "xyes" -a "x$use_libusb1" = "xyes"], [libftdi and libusb-1.x])
PROCESS_ADAPTERS([LIBGPIOD_ADAPTERS], ["x$use_libgpiod" = "xyes"], [Linux libgpiod])
PROCESS_ADAPTERS([DMEM_ADAPTER], ["x$is_linux" = "xyes"], [Linux /dev/mem])
PROCESS_ADAPTERS([SYSFSGPIO_ADAPTER], ["x$is_linux" = "xyes"], [Linux sysfs])
PROCESS_ADAPTERS([REMOTE_BITBANG_ADAPTER], [true], [unused])
PROCESS_ADAPTERS([LIBJAYLINK_ADAPTERS], ["x$use_internal_libjaylink" = "xyes" -o "x$use_libjaylink" = "xyes"], [libjaylink-0.2])
PROCESS_ADAPTERS([PCIE_ADAPTERS], ["x$is_linux" = "xyes" -a "x$ac_cv_header_linux_pci_h" = "xyes"], [Linux build])
PROCESS_ADAPTERS([SERIAL_PORT_ADAPTERS], ["x$can_build_buspirate" = "xyes"],
                                         [internal error: validation should happen beforehand])
PROCESS_ADAPTERS([LINUXSPIDEV_ADAPTER], ["x$is_linux" = "xyes" -a "x$ac_cv_header_linux_spi_spidev_h" = "xyes"],
                                        [Linux spidev])
PROCESS_ADAPTERS([VDEBUG_ADAPTER], [true], [unused])
PROCESS_ADAPTERS([JTAG_DPI_ADAPTER], [true], [unused])
PROCESS_ADAPTERS([JTAG_VPI_ADAPTER], [true], [unused])
PROCESS_ADAPTERS([RSHIM_ADAPTER], ["x$can_build_rshim" = "xyes"],
                                  [internal error: validation should happen beforehand])
PROCESS_ADAPTERS([AMTJTAGACCEL_ADAPTER], [true], [unused])
PROCESS_ADAPTERS([HOST_ARM_BITBANG_ADAPTERS], [true], [unused])
PROCESS_ADAPTERS([HOST_ARM_OR_AARCH64_BITBANG_ADAPTERS], [true], [unused])
PROCESS_ADAPTERS([DUMMY_ADAPTER], [true], [unused])

AS_IF([test "x$enable_linuxgpiod" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_sysfsgpio" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_remote_bitbang" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_bcm2835gpio" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_imx_gpio" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_am335xgpio" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_ep93xx" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_at91rm9200" != "xno"], [
  build_bitbang=yes
])

AS_IF([test "x$enable_stlink" != "xno" -o "x$enable_ti_icdi" != "xno" -o "x$enable_nulink" != "xno"], [
	AC_DEFINE([BUILD_HLADAPTER], [1], [1 if you want the High Level JTAG driver.])
	AM_CONDITIONAL([HLADAPTER], [true])
], [
	AC_DEFINE([BUILD_HLADAPTER], [0], [0 if you want the High Level JTAG driver.])
	AM_CONDITIONAL([HLADAPTER], [false])
])
AM_CONDITIONAL([HLADAPTER_STLINK], [test "x$enable_stlink" != "xno"])
AM_CONDITIONAL([HLADAPTER_ICDI], [test "x$enable_ti_icdi" != "xno"])
AM_CONDITIONAL([HLADAPTER_NULINK], [test "x$enable_nulink" != "xno"])

AS_IF([test "x$enable_jlink" != "xno"], [
  AS_IF([test "x$use_internal_libjaylink" = "xyes"], [
    AS_IF([test -f "$srcdir/src/jtag/drivers/libjaylink/configure.ac"], [
      AX_CONFIG_SUBDIR_OPTION([src/jtag/drivers/libjaylink],
		[--enable-subproject-build])
    ], [
      AC_MSG_ERROR([Internal libjaylink not found, run 'git submodule init' and 'git submodule update'.])
    ])
  ])
])

# Presto needs the bitq module
AS_IF([test "x$enable_presto" != "xno"], [
  build_bitq=yes
])

# esp-usb-jtag also needs the bitq module
AS_IF([test "x$enable_esp_usb_jtag" != "xno"], [
  build_bitq=yes
])

AM_CONDITIONAL([RELEASE], [test "x$build_release" = "xyes"])
AM_CONDITIONAL([PARPORT], [test "x$build_parport" = "xyes"])
AM_CONDITIONAL([GIVEIO], [test "x$parport_use_giveio" = "xyes"])
AM_CONDITIONAL([BITBANG], [test "x$build_bitbang" = "xyes"])
AM_CONDITIONAL([USB_BLASTER_DRIVER], [test "x$enable_usb_blaster" != "xno" -o "x$enable_usb_blaster_2" != "xno"])
AM_CONDITIONAL([GW16012], [test "x$build_gw16012" = "xyes"])
AM_CONDITIONAL([USE_LIBUSB1], [test "x$use_libusb1" = "xyes"])
AM_CONDITIONAL([IS_CYGWIN], [test "x$is_cygwin" = "xyes"])
AM_CONDITIONAL([IS_MINGW], [test "x$is_mingw" = "xyes"])
AM_CONDITIONAL([IS_WIN32], [test "x$is_win32" = "xyes"])
AM_CONDITIONAL([IS_DARWIN], [test "x$is_darwin" = "xyes"])
AM_CONDITIONAL([BITQ], [test "x$build_bitq" = "xyes"])
AM_CONDITIONAL([USE_LIBFTDI], [test "x$use_libftdi" = "xyes"])
AM_CONDITIONAL([USE_LIBGPIOD], [test "x$use_libgpiod" = "xyes"])
AM_CONDITIONAL([USE_HIDAPI], [test "x$use_hidapi" = "xyes"])
AM_CONDITIONAL([USE_LIBJAYLINK], [test "x$use_libjaylink" = "xyes"])
AM_CONDITIONAL([HAVE_CAPSTONE], [test "x$enable_capstone" != "xno"])

AM_CONDITIONAL([INTERNAL_JIMTCL], [test "x$use_internal_jimtcl" = "xyes"])
AM_CONDITIONAL([HAVE_JIMTCL_PKG_CONFIG], [test "x$have_jimtcl_pkg_config" = "xyes"])
AM_CONDITIONAL([INTERNAL_LIBJAYLINK], [test "x$use_internal_libjaylink" = "xyes"])

AM_CONDITIONAL([BUILD_ESP_REMOTE], [test "x$build_esp_remote" = "xyes"])
AM_CONDITIONAL([BUILD_ESP_COMPRESSION], [test "x$build_esp_compression" = "xyes"])

AM_CONDITIONAL([USE_GCOV], [test "x$enable_gcov" = "xyes"])

# Look for environ alternatives.  Possibility #1: is environ in unistd.h or stdlib.h?
AC_MSG_CHECKING([for environ in unistd.h and stdlib.h])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#define _GNU_SOURCE
#include <unistd.h>
#include <stdlib.h>
  ]], [[char **ep = environ;]]
  )], [
    AC_MSG_RESULT([yes])
    has_environ=yes
  ], [
    AC_MSG_RESULT([no])

    # Possibility #2: can environ be found in an available library?
    AC_MSG_CHECKING([for extern environ])
    AC_LINK_IFELSE([AC_LANG_PROGRAM([[
    extern char **environ;
    ]], [[char **ep = environ;]]
      )], [
        AC_DEFINE(NEED_ENVIRON_EXTERN, [1], [Must declare 'environ' to use it.])
        has_environ=yes
      ], [
        has_environ=no
      ])
    AC_MSG_RESULT([${has_environ}])
  ])

AS_IF([test "x${has_environ}" != "xyes" ], [
  AC_MSG_FAILURE([Could not find 'environ' in unistd.h or available libraries.])
])

AC_DEFINE([_GNU_SOURCE],[1],[Use GNU C library extensions (e.g. stdndup).])

# set default gcc warnings
GCC_WARNINGS="-Wall -Wstrict-prototypes -Wformat-security -Wshadow"
AS_IF([test "x${gcc_wextra}" = "xyes"], [
  GCC_WARNINGS="${GCC_WARNINGS} -Wextra -Wno-unused-parameter"
  GCC_WARNINGS="${GCC_WARNINGS} -Wbad-function-cast"
  GCC_WARNINGS="${GCC_WARNINGS} -Wcast-align"
  GCC_WARNINGS="${GCC_WARNINGS} -Wredundant-decls"
  GCC_WARNINGS="${GCC_WARNINGS} -Wpointer-arith"
  GCC_WARNINGS="${GCC_WARNINGS} -Wundef"
])
AS_IF([test "x${gcc_werror}" = "xyes"], [
  GCC_WARNINGS="${GCC_WARNINGS} -Werror"
])

# override default gcc cflags
AS_IF([test "x$gcc_warnings" = "xyes"], [
  AC_SUBST([GCC_WARNINGS], [$GCC_WARNINGS])
])

AC_SUBST(EXTRA_DIST_NEWS, ["$(echo $srcdir/NEWS-*)"])

AC_CONFIG_FILES([
  Makefile \
  testing/Makefile \
  testing/tcl_commands/Makefile
])
AC_OUTPUT

AS_IF([test "x$enable_jlink" != "xno"], [
  AS_IF([test "x$use_internal_libjaylink" = "xyes"], [
    AC_MSG_WARN([Using the internal libjaylink is deprecated and will not be possible in the future.])
  ]])
)

AS_IF([test "x$use_internal_jimtcl" = "xyes"], [
  AC_MSG_WARN([Using the internal jimtcl is deprecated and will not be possible in the future.])
])

AS_IF([test "x$enable_amtjtagaccel" != "xno"], [
  echo
  echo
  AC_MSG_WARN([Amontec JTAG-Accelerator adapter is deprecated and support will be removed in the next release!])
])
AS_IF([test "x$build_gw16012" = "xyes"], [
  echo
  echo
  AC_MSG_WARN([Gateworks GW16012 JTAG adapter is deprecated and support will be removed in the next release!])
])

echo
echo
echo OpenOCD configuration summary
echo ---------------------------------------------------
m4_foreach([adapterTuple], [USB1_ADAPTERS,
	HIDAPI_ADAPTERS, HIDAPI_USB1_ADAPTERS, LIBFTDI_ADAPTERS,
	LIBFTDI_USB1_ADAPTERS,
	LIBGPIOD_ADAPTERS,
	DMEM_ADAPTER,
	SYSFSGPIO_ADAPTER,
	REMOTE_BITBANG_ADAPTER,
	LIBJAYLINK_ADAPTERS, PCIE_ADAPTERS, SERIAL_PORT_ADAPTERS,
	LINUXSPIDEV_ADAPTER,
	VDEBUG_ADAPTER,
	JTAG_DPI_ADAPTER,
	JTAG_VPI_ADAPTER,
	RSHIM_ADAPTER,
	AMTJTAGACCEL_ADAPTER,
	HOST_ARM_BITBANG_ADAPTERS,
	HOST_ARM_OR_AARCH64_BITBANG_ADAPTERS,
	DUMMY_ADAPTER,
	OPTIONAL_LIBRARIES,
	COVERAGE],
	[s=m4_format(["%-49s"], ADAPTER_DESC([adapterTuple]))
	AS_CASE([$ADAPTER_VAR([adapterTuple])],
		[auto], [
			echo "$s"yes '(auto)'
		],
		[yes], [
			echo "$s"yes
		],
		[no], [
			echo "$s"no
		],
		[
			AC_MSG_ERROR(m4_normalize([
				Error in [adapterTuple] "ADAPTER_ARG([adapterTuple])": Variable "ADAPTER_VAR([adapterTuple])"
				has invalid value "$ADAPTER_VAR([adapterTuple])".]))
	])
])
echo
