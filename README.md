[![OpenOCD EasyDevKits CI](https://github.com/EasyDevKits/openocd-easydevkits/actions/workflows/OpenOCD%20EasyDevKits%20CI.yml/badge.svg)](https://github.com/EasyDevKits/openocd-easydevkits/actions/workflows/OpenOCD%20EasyDevKits%20CI.yml)

# Welcome to OpenOCD for EasyDevKits!

**This repository is forked from [espressif/openocd-esp32](https://github.com/espressif/openocd-esp32) and contains modifications for working with [EasyDevKits](https://www.easydevkits.com/)**

If you need more information about OpenOCD please refer to the [main project](https://openocd.org/)

The EasyDevKits project integrates a target chip (in this case an ESP32) with a JTAG adapter chip on a single development board.
For an introduction you can visit [EasyDevKits](https://www.easydevkits.com/) or watch the introduction video on [Youtube](https://www.youtube.com/watch?v=Hq00uXbZy-M)

# OpenOCD Documentation

In addition to the in-tree documentation, the latest manuals may be
viewed online at the following URLs:

  OpenOCD User's Guide:
    http://openocd.org/doc/html/index.html

  OpenOCD Developer's Manual:
    http://openocd.org/doc/doxygen/html/index.html

These reflect the latest development versions, so the following section
introduces how to build the complete documentation from the package.

For more information, refer to these documents or contact the developers
by subscribing to the OpenOCD developer mailing list:

	openocd-devel@lists.sourceforge.net

# Building OpenOCD

The INSTALL file contains generic instructions for running 'configure'
and compiling the OpenOCD source code. That file is provided by
default for all GNU autotools packages. If you are not familiar with
the GNU autotools, then you should read those instructions first.

The remainder of this document tries to provide some instructions for
those looking for a quick-install.

## OpenOCD Dependencies

GCC or Clang is currently required to build OpenOCD. The developers
have begun to enforce strict code warnings (-Wall, -Werror, -Wextra,
and more) and use C99-specific features: inline functions, named
initializers, mixing declarations with code, and other tricks. While
it may be possible to use other compilers, they must be somewhat
modern and could require extending support to conditionally remove
GCC-specific extensions.

You'll also need:

- make
- libtool
- pkg-config >= 0.23 or pkgconf

OpenOCD uses jimtcl library; build from git can retrieve jimtcl as git
submodule.

Additionally, for building from git:

- autoconf >= 2.69
- automake >= 1.14
- texinfo >= 5.0

Optional USB-based adapter drivers need libusb-1.0.

Optional USB-Blaster, ASIX Presto and OpenJTAG interface adapter
drivers need:
  - libftdi: http://www.intra2net.com/en/developer/libftdi/index.php

Optional CMSIS-DAP adapter driver needs HIDAPI library.

Optional linuxgpiod adapter driver needs libgpiod library.

Optional J-Link adapter driver needs libjaylink library.

Optional ARM disassembly needs capstone library.

Optional development script checkpatch needs:

- perl
- python
- python-ply

## Permissions delegation

Running OpenOCD with root/administrative permissions is strongly
discouraged for security reasons.

For USB devices on GNU/Linux you should use the contrib/60-openocd.rules
file. It probably belongs somewhere in /etc/udev/rules.d, but
consult your operating system documentation to be sure. Do not forget
to add yourself to the "plugdev" group.

For parallel port adapters on GNU/Linux and FreeBSD please change your
"ppdev" (parport* or ppi*) device node permissions accordingly.

For parport adapters on Windows you need to run install_giveio.bat
(it's also possible to use "ioperm" with Cygwin instead) to give
ordinary users permissions for accessing the "LPT" registers directly.

## Compiling OpenOCD

To build OpenOCD, use the following sequence of commands:

  ```./bootstrap (when building from the git repository)```
  
  ```./configure [options]```
  
  ```make```
  
  ```sudo make install```

The 'configure' step generates the Makefiles required to build
OpenOCD, usually with one or more options provided to it. The first
'make' step will build OpenOCD and place the final executable in
'./src/'. The final (optional) step, ``make install'', places all of
the files in the required location.

To see the list of all the supported options, run
  ```./configure --help```

## Cross-compiling Options

Cross-compiling is supported the standard autotools way, you just need
to specify the cross-compiling target triplet in the --host option,
e.g. for cross-building for Windows 32-bit with MinGW on Debian:

  ```./configure --host=i686-w64-mingw32 [options]```

To make pkg-config work nicely for cross-compiling, you might need an
additional wrapper script as described at

  https://autotools.io/pkgconfig/cross-compiling.html

This is needed to tell pkg-config where to look for the target
libraries that OpenOCD depends on. Alternatively, you can specify
*_CFLAGS and *_LIBS environment variables directly, see "./configure
--help" for the details.

For a more or less complete script that does all this for you, see

  contrib/cross-build.sh

## Parallel Port Dongles

If you want to access the parallel port using the PPDEV interface you
have to specify both --enable-parport AND --enable-parport-ppdev, since
the later option is an option to the parport driver.

The same is true for the --enable-parport-giveio option, you have to
use both the --enable-parport AND the --enable-parport-giveio option
if you want to use giveio instead of ioperm parallel port access
method.


## Obtaining OpenOCD From GIT

You can download the current GIT version with a GIT client of your
choice from the main repository:

   git://git.code.sf.net/p/openocd/code

You may prefer to use a mirror:

   http://repo.or.cz/r/openocd.git
   git://repo.or.cz/openocd.git

Using the GIT command line client, you might use the following command
to set up a local copy of the current repository (make sure there is no
directory called "openocd" in the current directory):

   ```git clone git://git.code.sf.net/p/openocd/code openocd```

Then you can update that at your convenience using

   ```git pull```

There is also a gitweb interface, which you can use either to browse
the repository or to download arbitrary snapshots using HTTP:

   http://repo.or.cz/w/openocd.git

Snapshots are compressed tarballs of the source tree, about 1.3 MBytes
each at this writing.
