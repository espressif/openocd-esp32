#include "gen_ut_app.h"

#if CONFIG_IDF_TARGET_ARCH_XTENSA

#include <xtensa/coreasm.h>
#include <xtensa/corebits.h>
#include <xtensa/config/system.h>
#include "soc/soc.h"

#if (UT_IDF_VER_MAJOR >= 6)
#include <xtensa/config/xt_specreg.h>
#else
#define XT_REG_CCOUNT CCOUNT
#define XT_REG_CCOMPARE_2 CCOMPARE2
#define XT_REG_EXCSAVE_5 EXCSAVE_5
#endif

    .data
scratch_regs:
    .word 0
    .word 0

    .text
    .section .iram1,"ax"
    .global     xt_highint5
    .type       xt_highint5,@function
    .align      4
xt_highint5:
    /* save a2 and a3 */
    movi    a0, scratch_regs
    s32i    a2, a0, 0
    s32i    a3, a0, 4
    /* set CCOMPARE2 = CCOUNT + period */
    rsr     a2, XT_REG_CCOUNT
    movi    a3, 100000000
    add     a2, a2, a3
    wsr     a2, XT_REG_CCOMPARE_2
    /* restore registers and return */
    movi    a0, scratch_regs
    l32i    a2, a0, 0
    l32i    a3, a0, 4
    rsr     a0, XT_REG_EXCSAVE_5
    rfi     5

    .global     include_xt_highint5
include_xt_highint5:

#endif