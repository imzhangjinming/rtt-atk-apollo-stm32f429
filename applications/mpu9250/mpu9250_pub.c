#include <rtthread.h>

#define LOG_TAG              "mpu9250_pub"
#include <ulog.h>

#include "om.h"
#include "mpu6xxx.h"
#include "mpu6xxx_reg.h"
#include "mpu9250_pub.h"

#define MPU9250_PUB_THREAD_STACK_SIZE 1024
#define MPU9250_PUB_THREAD_PRIORITY 20
#define MPU9250_PUB_THREAD_TIMESLICE 20

#define MPU9250_MSG_BUF_SIZE 12

#define MPU9250_PUB_FREQ_HZ  250



static mpu9250_msg mpu9250_msg_buf;
om_topic_t mpu9250_topic;

static struct rt_completion completion;
static struct rt_timer mpu9250_pub_freq_timer;
static struct rt_timer mpu9250_pub_cnt_timer;


rt_align(RT_ALIGN_SIZE)

static struct rt_thread mpu9250_pub_thread;
static char mpu9250_pub_thread_stack[MPU9250_PUB_THREAD_STACK_SIZE];

extern struct mpu6xxx_device mpu9250_dev;

rt_uint32_t pub_cnt = 0;

void mpu9250_pub_freq_timer_cb(void *parameter)
{
    rt_completion_done(&completion);
}

void mpu9250_pub_cnt_cb(void *parameter)
{
    LOG_I("%d", pub_cnt);
    pub_cnt = 0;
}

void mpu9250_pub_thread_entry(void *parameter)
{
    rt_uint8_t buf[MPU9250_MSG_BUF_SIZE];
    rt_uint16_t fifo_cnt = 0;
    rt_err_t ret = RT_EOK;
    rt_uint8_t reg = 0x00;

    while(1)
    {
        rt_completion_wait(&completion, RT_WAITING_FOREVER);

        mpu6xxx_get_accel(&mpu9250_dev, &mpu9250_msg_buf.accel);
        mpu6xxx_get_gyro(&mpu9250_dev, &mpu9250_msg_buf.gyro);

        om_publish(&mpu9250_topic,
                    &mpu9250_msg_buf,
                    MPU9250_MSG_BUF_SIZE,
                    true,
                    false);
    
        pub_cnt++;
        // LOG_I("%d", pub_cnt);
    }
}

rt_err_t mpu9250_pub_init(void)
{
    rt_err_t ret = RT_EOK;

    rt_completion_init(&completion);

    om_create_topic_static(&mpu9250_topic, 
                            "mpu9250_topic", 
                            sizeof(mpu9250_msg));

    rt_tick_t period = rt_tick_from_millisecond(1000 / MPU9250_PUB_FREQ_HZ);

    rt_timer_init(&mpu9250_pub_cnt_timer,
                    "mpu9250_pub_cnt_timer",
                    mpu9250_pub_cnt_cb,
                    RT_NULL,
                    1000,
                    RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER);

    rt_timer_start(&mpu9250_pub_cnt_timer);

    rt_timer_init(&mpu9250_pub_freq_timer,
                  "mpu9250_pub_freq_timer",
                  mpu9250_pub_freq_timer_cb,
                  RT_NULL,
                  period,
                  RT_TIMER_FLAG_PERIODIC|RT_TIMER_FLAG_SOFT_TIMER);

    rt_timer_start(&mpu9250_pub_freq_timer);




    ret = rt_thread_init(&mpu9250_pub_thread,
                         "mpu9250_pub",
                         mpu9250_pub_thread_entry,
                         RT_NULL,
                         &mpu9250_pub_thread_stack[0],
                         sizeof(mpu9250_pub_thread_stack),
                         MPU9250_PUB_THREAD_PRIORITY,
                         MPU9250_PUB_THREAD_TIMESLICE);
    
    if(ret != RT_EOK)
    {
        LOG_E("init mpu9250_pub_thread failed");
        return RT_ERROR;
    }

    ret = rt_thread_startup(&mpu9250_pub_thread);

    return ret;
}
