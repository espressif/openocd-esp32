# SPDX-License-Identifier: GPL-2.0-or-later

add_library(xtensa STATIC)

target_sources(xtensa PRIVATE
    xtensa.c
    xtensa.h
    xtensa_chip.c
    xtensa_chip.h
    xtensa_debug_module.c
    xtensa_debug_module.h
    xtensa_fileio.c
    xtensa_fileio.h
    xtensa_regs.h
)
