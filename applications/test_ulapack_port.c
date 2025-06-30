/**
 * @name pca.c
 * An example of using uLAPAck for principle component analysis.
 *
 * @note In this example, static memory allocation is used
 *       via the -D compiler options.
 * @note Build this example using: $ make pca
 * @note Run this example using: $ ./pca
 */

#include <rtthread.h>
#include <ulog.h>
#include <stdio.h>


#include "ulapack.h"

#define ULAPACK_TEST_STACK_SIZE 4096
#define ULAPACK_TEST_PRIORITY 21
#define ULAPACK_TEST_TIMESLICE 20

rt_align(RT_ALIGN_SIZE);

static char ulapack_test_stack[ULAPACK_TEST_STACK_SIZE];
static struct rt_thread ulapack_test_thread;

rt_err_t test_ulapack_port_main(void) {
    double Adata[2][2] = {  {0.4170, 0.4192},
                            {0.7203, 0.6852}};
    Matrix_t A;
    Matrix_t T;

    ulapack_init(&A, 2, 2);
    ulapack_init(&T, 2, 2);

    /*
     * Copy data points into vector objects.
     */
    for (Index_t row_itor = 0; row_itor < 2; row_itor++) {
        for (Index_t col_itor = 0; col_itor < 2; col_itor++) {
            ulapack_edit_entry(&A, 
            row_itor, col_itor, 
            Adata[row_itor][col_itor]);
        }
    }

    ulapack_pca(&A, &T);

    rt_kprintf("\nA = \n");
    ulapack_print(&A, stdout);

    rt_kprintf("\nT = \n");
    ulapack_print(&T, stdout);

    return RT_EOK;
}

static void ulapack_test_thread_entry(void *parameter) {
    rt_err_t ret = RT_EOK;

    ret = test_ulapack_port_main();

    if (ret != RT_EOK) {
        LOG_E("test_ulapack_port_main failed");
    }

    while(1)
    {
        rt_thread_mdelay(1000);
    }
}

void ulapack_test(void)
{
    rt_thread_init(&ulapack_test_thread,
                    "ulapack_test",
                    ulapack_test_thread_entry,
                    RT_NULL,
                    &ulapack_test_stack[0],
                    sizeof(ulapack_test_stack),
                    ULAPACK_TEST_PRIORITY,
                    ULAPACK_TEST_TIMESLICE);
    rt_thread_startup(&ulapack_test_thread);
}

