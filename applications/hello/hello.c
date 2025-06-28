#include <stdio.h>
#include <finsh.h>
#include <rtthread.h>

#include "hello.h"

int hello_world(void)
{
    rt_kprintf("Hello RT-Thread!\n");
    return 0;
}

MSH_CMD_EXPORT(hello_world, hello world);