#!/bin/bash
src/openocd -s tcl -f board/esprv_ftdi.cfg

#Use the following command to download via ocd
#src/openocd -s tcl -f board/esprv_ftdi.cfg -c "init; esp32_attach_proc_pro2"
