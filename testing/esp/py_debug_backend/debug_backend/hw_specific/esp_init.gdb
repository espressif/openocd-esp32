echo [default ESP gdbinit file]
set remotetimeout 3
set remote hardware-watchpoint-limit 2
mon reset halt
flushregs
