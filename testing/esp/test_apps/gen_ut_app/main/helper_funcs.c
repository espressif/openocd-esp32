#include <stdio.h>
#include "sdkconfig.h"


#if CONFIG_ESP32_GCOV_ENABLE
void gcov_dummy_func(void)
{
    static int i;
    printf("Counter = %d\n", i++);
}
#endif
