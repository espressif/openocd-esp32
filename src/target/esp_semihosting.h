#ifndef _ESP_SEMIHOSTING_H_
#define _ESP_SEMIHOSTING_H_

#define ESP_SYS_DRV_INFO_LEGACY                     0xE0

#define ESP_SEMIHOSTING_SYS_DRV_INFO                0x100
#define ESP_SEMIHOSTING_SYS_APPTRACE_INIT           0x101
#define ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT        0x102
#define ESP_SEMIHOSTING_SYS_BREAKPOINT_SET          0x103
#define ESP_SEMIHOSTING_SYS_WATCHPOINT_SET          0x104
#define ESP_SEMIHOSTING_SYS_MKDIR                   0x105
#define ESP_SEMIHOSTING_SYS_RMDIR                   0x106
#define ESP_SEMIHOSTING_SYS_ACCESS                  0x107
#define ESP_SEMIHOSTING_SYS_TRUNCATE                0x108
#define ESP_SEMIHOSTING_SYS_UTIME                   0x109
#define ESP_SEMIHOSTING_SYS_FSTAT                   0x10A
#define ESP_SEMIHOSTING_SYS_STAT                    0x10B
#define ESP_SEMIHOSTING_SYS_FSYNC                   0x10C
#define ESP_SEMIHOSTING_SYS_LINK                    0x10D
#define ESP_SEMIHOSTING_SYS_UNLINK                  0x10E
#define ESP_SEMIHOSTING_SYS_OPENDIR                 0x10F
#define ESP_SEMIHOSTING_SYS_CLOSEDIR                0x110
#define ESP_SEMIHOSTING_SYS_READDIR                 0x111
#define ESP_SEMIHOSTING_SYS_READDIR_R               0x112
#define ESP_SEMIHOSTING_SYS_SEEKDIR                 0x113
#define ESP_SEMIHOSTING_SYS_TELLDIR                 0x114
#define ESP_SEMIHOSTING_SYS_SEEK                    0x115   /* custom lseek with whence parameter */

#endif