#include <xtensa/coreasm.h>
#include <xtensa/corebits.h>
#include <xtensa/config/system.h>
#include "sdkconfig.h"
#include "soc/soc.h"

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
    rsr     a2, CCOUNT
    movi    a3, 100000000
    add     a2, a2, a3
    wsr     a2, CCOMPARE2
    /* restore registers and return */
    movi    a0, scratch_regs
    l32i    a2, a0, 0
    l32i    a3, a0, 4
    rsr     a0, EXCSAVE_5
    rfi     5

    .global     include_xt_highint5
include_xt_highint5:
