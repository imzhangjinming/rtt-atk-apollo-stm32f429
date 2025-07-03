/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <ulog.h>

#include "om.h"
#include "mpu6xxx.h"

#define LED0_PIN    GET_PIN(B, 1)

extern rt_err_t mpu9250_pub_init(void);
extern void mpu9250_sub_test(void);
// extern void ulapack_test(void);
extern rt_err_t ekf_mpu9250_init(void);


int main(void)
{
    ulog_init();

    rt_err_t ret = RT_EOK;

    ret = om_init();
    if (ret != OM_OK)
    {
        LOG_E("om_init failed");
    }

    ret = mpu9250_pub_init();
    if (ret != RT_EOK)
    {
        LOG_E("mpu9250_pub_init failed");
    }

    mpu6xxx_init("i2c1", RT_NULL);

    // mpu9250_sub_test();

    // ulapack_test();

    ekf_mpu9250_init();

    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
