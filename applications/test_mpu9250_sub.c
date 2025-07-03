#include <rtthread.h>
#include <ulog.h>
#include "om.h"
#include "mpu9250_pub.h"

#define MPU9250_SUB_TEST_STACK_SIZE 1024
#define MPU9250_SUB_TEST_PRIORITY 21
#define MPU9250_SUB_TEST_TIMESLICE 20

extern om_topic_t mpu9250_topic;
static om_suber_t mpu9250_test_suber;

rt_align(RT_ALIGN_SIZE);

static char mpu9250_sub_test_stack[MPU9250_SUB_TEST_STACK_SIZE];
static struct rt_thread mpu9250_sub_test_thread;

static void mpu9250_sub_test_entry(void *parameter)
{
    om_create_suber_static(&mpu9250_test_suber,
                            &mpu9250_topic);
    om_subscribe_static(&mpu9250_topic,
                        &mpu9250_test_suber);
    
    rt_uint32_t recv_cnter = 0;
    mpu9250_msg msg;
    om_status_t ret = OM_OK;

    while(1)
    {
        if(om_suber_available(&mpu9250_test_suber))
        {
            ret = om_suber_export(&mpu9250_test_suber, 
                                    &msg,
                                    false);
            if(ret == OM_OK)
            {
                // LOG_I("ax = %4d, ay = %4d, az = %4d, gx = %4d, gy = %4d, gz = %4d", msg.accel.x, msg.accel.y, msg.accel.z, msg.gyro.x, msg.gyro.y, msg.gyro.z);
                rt_kprintf("%8d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d\n", 
                                rt_tick_get(), 
                                msg.accel.x, msg.accel.y, msg.accel.z, 
                                msg.gyro.x, msg.gyro.y, msg.gyro.z, 
                                msg.mag.x, msg.mag.y, msg.mag.z);
            }
            else
            {
                LOG_E("om_suber_export failed , err code: %d", ret);
            }
        }
    }
}

void mpu9250_sub_test(void)
{
    rt_thread_init(&mpu9250_sub_test_thread,
                    "mpu9250_sub_test",
                    mpu9250_sub_test_entry,
                    RT_NULL,
                    &mpu9250_sub_test_stack[0],
                    sizeof(mpu9250_sub_test_stack),
                    MPU9250_SUB_TEST_PRIORITY,
                    MPU9250_SUB_TEST_TIMESLICE);
    rt_thread_startup(&mpu9250_sub_test_thread);
}