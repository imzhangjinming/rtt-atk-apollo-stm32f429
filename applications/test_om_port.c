#include <rtthread.h>
#include <ulog.h>
#include "om.h"

#define OM_PORT_TEST_STACK_SIZE 1024
#define OM_PORT_TEST_PRIORITY 20
#define OM_PORT_TEST_TIMESLICE 10

rt_align(RT_ALIGN_SIZE);

static char om_port_test_stack[OM_PORT_TEST_STACK_SIZE];
static struct rt_thread om_port_test_thread;
static char om_port_test_recv_stack[OM_PORT_TEST_STACK_SIZE];
static struct rt_thread om_port_test_recv_thread;

static om_topic_t counter_test_topic;
static om_suber_t counter_test_suber;

static void om_port_test_entry(void *parameter)
{
    rt_uint32_t counter = 0;
    om_create_topic_static(&counter_test_topic, 
                            "counter_test_publish", 
                            sizeof(rt_uint32_t));
    
    while(1)
    {
        if(om_publish(&counter_test_topic, &counter, sizeof(rt_uint32_t), true, false) == OM_OK)
        {
            LOG_I("publish counter: %d", counter);
        }
        else
        {
            LOG_E("publish counter failed");
        }
        rt_thread_mdelay(1000);
        counter++;
    }
}


static void om_port_test_recv_entry(void *parameter)
{
    om_create_suber_static(&counter_test_suber,
                            &counter_test_topic);
    om_subscribe_static(&counter_test_topic,
                        &counter_test_suber);
    
    rt_uint32_t recv_cnter = 0;
    om_status_t ret = OM_OK;

    while(1)
    {
        if(om_suber_available(&counter_test_suber))
        {
            ret = om_suber_export(&counter_test_suber, 
                            &recv_cnter,
                            false);
            if(ret == OM_OK)
            {
                LOG_I("recv counter: %d", recv_cnter);
            }
            else
            {
                LOG_E("recv counter failed, err code: %d", ret);
            }
        }
    }
}

rt_err_t rt_om_init(void)
{
    rt_err_t ret = RT_EOK;

    ret = om_init();
    if (ret != OM_OK)
    {
        LOG_E("om_init failed");
        return ret;
    }

    LOG_I("om_init success");
    return RT_EOK;
}

void om_port_test(void)
{
    rt_om_init();

    rt_thread_init(&om_port_test_recv_thread,
                   "om_port_test_recv",
                   om_port_test_recv_entry,
                   RT_NULL,
                   &om_port_test_recv_stack[0],
                   sizeof(om_port_test_recv_stack),
                   OM_PORT_TEST_PRIORITY,
                   OM_PORT_TEST_TIMESLICE);

    rt_thread_startup(&om_port_test_recv_thread);

    rt_thread_init(&om_port_test_thread,
                   "om_port_test",
                   om_port_test_entry,
                   RT_NULL,
                   &om_port_test_stack[0],
                   sizeof(om_port_test_stack),
                   OM_PORT_TEST_PRIORITY,
                   OM_PORT_TEST_TIMESLICE);

    rt_thread_startup(&om_port_test_thread);

}




