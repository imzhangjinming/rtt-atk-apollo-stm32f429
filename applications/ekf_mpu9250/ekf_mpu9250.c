#include <rtthread.h>

#define LOG_TAG "ekf_mpu9250"
#include <ulog.h>
#include <stdio.h>

#include "om.h"
#include "mpu9250_pub.h"
#include "ulapack.h"

#define EKF_MPU9250_STACK_SIZE (1024*32)
#define EKF_MPU9250_PRIORITY  20
#define EKF_MPU9250_TIMESLICE  50

#define EKF_MPU9250_G (980.665f)
#define EKF_MPU9250_RAD_TO_DEG (57.2958f)

extern om_topic_t mpu9250_topic;
static om_suber_t ekf_mpu9250_suber;


rt_align(RT_ALIGN_SIZE)

static struct rt_thread ekf_mpu9250_thread;
static char ekf_mpu9250_thread_stack[EKF_MPU9250_STACK_SIZE];

volatile static double prev = 0.0;
volatile static double dt = 0.0;

static Matrix_t accel;
static Matrix_t gyro;
static Matrix_t mag;
static Matrix_t omega;
static Matrix_t A;
static Matrix_t P_pri;
static Matrix_t P_post;
static Matrix_t I_4;
static Matrix_t q_pri;
static Matrix_t q_cor;
static Matrix_t q_post;
static Matrix_t H1;
static Matrix_t H2;
static Matrix_t K;
static Matrix_t h1;
static Matrix_t h2;

static Matrix_t Q;
static Matrix_t R1;
static Matrix_t R2;

static Matrix_t euler;

static Matrix_t Ta;
static Matrix_t Ka;
static Matrix_t Ba;

static Matrix_t Tg;
static Matrix_t Kg;
static Matrix_t Bg;

static Matrix_t Tm2a;
static Matrix_t Bm;

static Matrix_t Vm;


static rt_err_t copy_ulapack_matrix(Matrix_t *dst, MatrixEntry_t **src, Index_t rows, Index_t cols)
{
    if(dst == RT_NULL || src == RT_NULL)
    {
        LOG_E("copy_ulapack_matrix failed: dst or src is null");
        return RT_ERROR;
    }

    if(dst->n_rows!=rows || dst->n_cols!=cols)
    {
        LOG_E("copy_ulapack_matrix failed: dst and src size is not match");
        return RT_ERROR;
    }

    for(Index_t i = 0; i < rows; i++)
    {
        for(Index_t j = 0; j < cols; j++)
        {
            dst->entry[i][j] = src[i][j];
        }
    }
    return RT_EOK;
}

static inline void ekf_mpu9250_compose_omega(void)
{
    ulapack_edit_entry(&omega, 0, 1, -gyro.entry[0][0]);
    ulapack_edit_entry(&omega, 0, 2, -gyro.entry[1][0]);
    ulapack_edit_entry(&omega, 0, 3, -gyro.entry[2][0]);

    ulapack_edit_entry(&omega, 1, 0, gyro.entry[0][0]);
    ulapack_edit_entry(&omega, 1, 2, gyro.entry[2][0]);
    ulapack_edit_entry(&omega, 1, 3, -gyro.entry[1][0]);

    ulapack_edit_entry(&omega, 2, 0, gyro.entry[1][0]);
    ulapack_edit_entry(&omega, 2, 1, -gyro.entry[2][0]);
    ulapack_edit_entry(&omega, 2, 3, gyro.entry[0][0]);

    ulapack_edit_entry(&omega, 3, 0, gyro.entry[2][0]);
    ulapack_edit_entry(&omega, 3, 1, gyro.entry[1][0]);
    ulapack_edit_entry(&omega, 3, 2, -gyro.entry[0][0]);
}

static inline void ekf_mpu9250_compose_jacobian_1(Matrix_t* H, Matrix_t* q)
{
    ulapack_edit_entry(H, 0, 0, -2.0*q->entry[2][0]);
    ulapack_edit_entry(H, 0, 1, 2.0*q->entry[3][0]);
    ulapack_edit_entry(H, 0, 2, -2.0*q->entry[0][0]);
    ulapack_edit_entry(H, 0, 3, 2.0*q->entry[1][0]);

    ulapack_edit_entry(H, 1, 0, 2.0*q->entry[1][0]);
    ulapack_edit_entry(H, 1, 1, 2.0*q->entry[0][0]);
    ulapack_edit_entry(H, 1, 2, 2.0*q->entry[3][0]);
    ulapack_edit_entry(H, 1, 3, 2.0*q->entry[2][0]);

    ulapack_edit_entry(H, 2, 0, 2.0*q->entry[0][0]);
    ulapack_edit_entry(H, 2, 1, -2.0*q->entry[1][0]);
    ulapack_edit_entry(H, 2, 2, -2.0*q->entry[2][0]);
    ulapack_edit_entry(H, 2, 3, 2.0*q->entry[3][0]);
}

inline static void ekf_mpu9250_compose_jacobian_2(Matrix_t* H, Matrix_t* q)
{
    ulapack_edit_entry(H, 0, 0, 2.0*q->entry[3][0]);
    ulapack_edit_entry(H, 0, 1, 2.0*q->entry[2][0]);
    ulapack_edit_entry(H, 0, 2, 2.0*q->entry[1][0]);
    ulapack_edit_entry(H, 0, 3, 2.0*q->entry[0][0]);

    ulapack_edit_entry(H, 1, 0, 2.0*q->entry[0][0]);
    ulapack_edit_entry(H, 1, 1, -2.0*q->entry[1][0]);
    ulapack_edit_entry(H, 1, 2, 2.0*q->entry[2][0]);
    ulapack_edit_entry(H, 1, 3, -2.0*q->entry[3][0]);

    ulapack_edit_entry(H, 2, 0, -2.0*q->entry[1][0]);
    ulapack_edit_entry(H, 2, 1, -2.0*q->entry[0][0]);
    ulapack_edit_entry(H, 2, 2, 2.0*q->entry[3][0]);
    ulapack_edit_entry(H, 2, 3, 2.0*q->entry[2][0]);
}

static inline void ekf_mpu9250_compose_g(Matrix_t* q)
{
    ulapack_edit_entry(&h1, 0, 0, 
                        2.0*q->entry[1][0]*q->entry[3][0] 
                        - 2.0 * q->entry[0][0]*q->entry[2][0]);

    ulapack_edit_entry(&h1, 1, 0, 
                        2.0*q->entry[0][0]*q->entry[1][0]
                        + 2.0 * q->entry[2][0]*q->entry[3][0]);

    ulapack_edit_entry(&h1, 2, 0, 
                        q->entry[0][0]*q->entry[0][0]
                        - q->entry[1][0]*q->entry[1][0]
                        - q->entry[2][0]*q->entry[2][0]
                        + q->entry[3][0]*q->entry[3][0]);
}

static inline void ekf_mpu9250_compose_mag(Matrix_t* q)
{
    ulapack_edit_entry(&h2, 0, 0, 
                        2.0*q->entry[1][0]*q->entry[2][0] 
                        + 2.0 * q->entry[0][0]*q->entry[3][0]);

    ulapack_edit_entry(&h2, 1, 0, 
                        q->entry[0][0]*q->entry[0][0]
                        - q->entry[1][0]*q->entry[1][0]
                        + q->entry[2][0]*q->entry[2][0]
                        - q->entry[3][0]*q->entry[3][0]);

    ulapack_edit_entry(&h2, 2, 0, 
                        2.0*q->entry[2][0]*q->entry[3][0]
                        - 2.0 * q->entry[0][0]*q->entry[1][0]);
}

static inline void ekf_mpu9250_quaternion_to_euler(Matrix_t* q, Matrix_t* euler)
{
    ulapack_edit_entry(euler, 0, 0, 
                        atan2(2.0*q->entry[0][0]*q->entry[1][0] 
                                + 2.0*q->entry[2][0]*q->entry[3][0], 
                                -2.0*q->entry[1][0]*q->entry[1][0] 
                                - 2.0*q->entry[2][0]*q->entry[2][0] + 1.0));

    ulapack_edit_entry(euler, 1, 0, 
                        asin(-2.0*q->entry[1][0]*q->entry[3][0] 
                                + 2.0*q->entry[0][0]*q->entry[2][0]));

    ulapack_edit_entry(euler, 2, 0, 
                        atan2(2.0*q->entry[0][0]*q->entry[3][0] 
                                + 2.0*q->entry[2][0]*q->entry[1][0], 
                                - 2.0*q->entry[2][0]*q->entry[2][0] 
                                - 2.0*q->entry[3][0]*q->entry[3][0] + 1.0));
}

static inline void ekf_mpu9250_init_P_post(void)
{
    ulapack_set(&P_post, 0.01*0.001);
    ulapack_edit_entry(&P_post, 0, 0, 0.1*0.001);
    ulapack_edit_entry(&P_post, 1, 1, 0.1*0.001);
    ulapack_edit_entry(&P_post, 2, 2, 0.1*0.001);
    ulapack_edit_entry(&P_post, 3, 3, 0.1*0.001);
}


static void ekf_mpu9250_thread_entry(void *parameter)
{
    om_create_suber_static(&ekf_mpu9250_suber, &mpu9250_topic);
    om_subscribe_static(&mpu9250_topic, &ekf_mpu9250_suber);

    mpu9250_msg mpu9250_msg_buf;
    om_status_t ret = OM_OK;


    ulapack_init(&accel, 3, 1);
    ulapack_init(&gyro, 3, 1);
    ulapack_init(&mag, 3, 1);

    ulapack_init(&omega, 4, 4);
    ulapack_init(&A, 4, 4);
    ulapack_init(&P_pri, 4, 4);
    ulapack_init(&P_post, 4, 4);

    ekf_mpu9250_init_P_post();

    ulapack_init(&I_4, 4, 4);
    ulapack_eye(&I_4);
    ulapack_init(&q_pri, 4, 1);
    ulapack_init(&q_cor, 4, 1);
    ulapack_init(&q_post, 4, 1);
    ulapack_edit_entry(&q_post, 0, 0, 1.0);
    ulapack_init(&H1, 3, 4);
    ulapack_init(&H2, 3, 4);
    ulapack_init(&K, 4, 3);
    ulapack_init(&h1, 3, 1);
    ulapack_init(&h2, 3, 1);
    
    ulapack_init(&Q, 4, 4);
    ulapack_init(&R1, 3, 3);
    ulapack_init(&R2, 3, 3);

    ulapack_init(&euler, 3, 1);

    ulapack_edit_entry(&Q, 0, 0, 0.0001);
    ulapack_edit_entry(&Q, 1, 1, 0.0001);
    ulapack_edit_entry(&Q, 2, 2, 0.0001);
    ulapack_edit_entry(&Q, 3, 3, 0.0001);

    ulapack_edit_entry(&R1, 0, 0, 0.1);
    ulapack_edit_entry(&R1, 1, 1, 0.1);
    ulapack_edit_entry(&R1, 2, 2, 0.1);

    ulapack_edit_entry(&R2, 0, 0, 0.5);
    ulapack_edit_entry(&R2, 1, 1, 0.5);
    ulapack_edit_entry(&R2, 2, 2, 0.5);

    ulapack_init(&Ta, 3, 3);
    ulapack_edit_entry(&Ta, 0, 0, 1.0);
    ulapack_edit_entry(&Ta, 0, 1, -0.0065);
    ulapack_edit_entry(&Ta, 0, 2, -0.0025);
    ulapack_edit_entry(&Ta, 1, 1, 1.0);
    ulapack_edit_entry(&Ta, 1, 2, -0.0076);
    ulapack_edit_entry(&Ta, 2, 2, 1.0);

    ulapack_init(&Ka, 3, 3);
    ulapack_edit_entry(&Ka, 0, 0, 0.0098);
    ulapack_edit_entry(&Ka, 1, 1, 0.0098);
    ulapack_edit_entry(&Ka, 2, 2, 0.0097);

    ulapack_init(&Ba, 3, 1);
    ulapack_edit_entry(&Ba, 0, 0, -6.7921);
    ulapack_edit_entry(&Ba, 1, 0, -19.3155);
    ulapack_edit_entry(&Ba, 2, 0, -89.7918);

    ulapack_init(&Tg, 3, 3);
    ulapack_edit_entry(&Tg, 0, 0, 1.0);
    ulapack_edit_entry(&Tg, 0, 1, -0.0057);
    ulapack_edit_entry(&Tg, 0, 2, 0.0069);
    ulapack_edit_entry(&Tg, 1, 0, 0.002);
    ulapack_edit_entry(&Tg, 1, 1, 1.0);
    ulapack_edit_entry(&Tg, 1, 2, 0.007);
    ulapack_edit_entry(&Tg, 2, 0, -0.0055);
    ulapack_edit_entry(&Tg, 2, 1, 0.0196);
    ulapack_edit_entry(&Tg, 2, 2, 1.0);

    ulapack_init(&Kg, 3, 3);
    ulapack_edit_entry(&Kg, 0, 0, 0.0174);
    ulapack_edit_entry(&Kg, 1, 1, 0.0176);
    ulapack_edit_entry(&Kg, 2, 2, 0.0192);

    ulapack_init(&Bg, 3, 1);
    ulapack_edit_entry(&Bg, 0, 0, -0.7848);
    ulapack_edit_entry(&Bg, 1, 0, -0.5321);
    ulapack_edit_entry(&Bg, 2, 0, 0.0072);

    ulapack_init(&Tm2a, 3, 3);
    ulapack_edit_entry(&Tm2a, 0, 0, -0.0644);
    ulapack_edit_entry(&Tm2a, 0, 1, 1.0709);
    ulapack_edit_entry(&Tm2a, 0, 2, -0.0283);
    ulapack_edit_entry(&Tm2a, 1, 0, 0.947);
    ulapack_edit_entry(&Tm2a, 1, 1, 0.0085);
    ulapack_edit_entry(&Tm2a, 1, 2, -0.0402);
    ulapack_edit_entry(&Tm2a, 2, 0, -0.0159);
    ulapack_edit_entry(&Tm2a, 2, 1, 0.0231);
    ulapack_edit_entry(&Tm2a, 2, 2, -1.0);

    ulapack_init(&Bm, 3, 1);
    ulapack_edit_entry(&Bm, 0, 0, -12.8627);
    ulapack_edit_entry(&Bm, 1, 0, 9.5862);
    ulapack_edit_entry(&Bm, 2, 0, 21.6452);

    ulapack_init(&Vm, 3, 1);
    ulapack_edit_entry(&Vm, 0, 0, 0.0);
    ulapack_edit_entry(&Vm, 1, 0, 0.6546);
    ulapack_edit_entry(&Vm, 2, 0, -0.7560);

    Matrix_t temp_4_4;
    ulapack_init(&temp_4_4, 4, 4);

    Matrix_t temp_4_3;
    ulapack_init(&temp_4_3, 4, 3);

    Matrix_t temp_3_3;
    ulapack_init(&temp_3_3, 3, 3);

    Matrix_t inv_temp_3_3;
    ulapack_init(&inv_temp_3_3, 3, 3);

    Matrix_t temp_3_4;
    ulapack_init(&temp_3_4, 3, 4);

    Matrix_t temp_3_1;
    ulapack_init(&temp_3_1, 3, 1);

    Matrix_t temp_4_1;
    ulapack_init(&temp_4_1, 4, 1);

    double norm_a = 0;
    double norm_gyro = 0;
    double norm_mag = 0;

    while(1)
    {
        if(om_suber_available(&ekf_mpu9250_suber))
        {
            ret = om_suber_export(&ekf_mpu9250_suber, 
                                    &mpu9250_msg_buf,
                                    false);
            
            rt_tick_t now = rt_tick_get_millisecond();
            dt = (now - prev) / 1000.0;


            ulapack_edit_entry(&accel, 0, 0, (double)mpu9250_msg_buf.accel.x);
            ulapack_edit_entry(&accel, 1, 0, (double)mpu9250_msg_buf.accel.y);
            ulapack_edit_entry(&accel, 2, 0, (double)mpu9250_msg_buf.accel.z);

            ulapack_edit_entry(&gyro, 0, 0, (double)mpu9250_msg_buf.gyro.x);    
            ulapack_edit_entry(&gyro, 1, 0, (double)mpu9250_msg_buf.gyro.y);    
            ulapack_edit_entry(&gyro, 2, 0, (double)mpu9250_msg_buf.gyro.z);    

            ulapack_edit_entry(&mag, 0, 0, (double)mpu9250_msg_buf.mag.x);
            ulapack_edit_entry(&mag, 1, 0, (double)mpu9250_msg_buf.mag.y);
            ulapack_edit_entry(&mag, 2, 0, (double)mpu9250_msg_buf.mag.z);

            ulapack_add(&accel, &Ba, &accel);
            ulapack_product(&Ka, &accel, &temp_3_1);
            ulapack_product(&Ta, &temp_3_1, &accel);

            ulapack_add(&gyro, &Bg, &gyro);
            ulapack_product(&Kg, &gyro, &temp_3_1);
            ulapack_product(&Tg, &temp_3_1, &gyro);

            ulapack_add(&mag, &Bm, &mag);
            ulapack_product(&Tm2a, &mag, &temp_3_1);
            ulapack_scale(&temp_3_1, 1.0, &mag);

            ulapack_norm(&accel, &norm_a);
            ulapack_norm(&gyro, &norm_gyro);
            ulapack_norm(&mag, &norm_mag);

            if (norm_a > (9.8+2) || norm_a < (9.8-2) || norm_gyro > 2)
            {
                // rt_kprintf("norm_a = %f\n", norm_a);
                continue;
            }
            else
            {
                prev = now;
                /* propagation step */ 
                ekf_mpu9250_compose_omega();

                // A = I_4 + 0.5 * dt * omega
                ulapack_scale(&omega, 0.5f*dt, &A);
                ulapack_add(&A, &I_4, &A);

                // rt_kprintf("A\n");
                // ulapack_print(&A, stdout);
                
                // q_pri = A * q_post
                ulapack_product(&A, &q_post, &q_pri);
                double temp_norm;
                ulapack_norm(&q_pri, &temp_norm);
                ulapack_scale(&q_pri, 1.0/temp_norm, &q_pri);
                
                // P_pri = A * P_post * A' + Q
                ulapack_product(&A, &P_post, &P_pri);
                ulapack_transpose(&A, &temp_4_4);
                Matrix_t temp_temp_4_4;
                ulapack_init(&temp_temp_4_4, 4, 4);
                ulapack_product(&P_pri, &temp_4_4, &temp_temp_4_4);
                ulapack_add(&temp_temp_4_4, &Q, &P_pri);


                /* update stage 1 */
                ekf_mpu9250_compose_jacobian_1(&H1, &q_pri);

                // K = P_pri * H' * (H * P_pri * H' + R)^-1
                ulapack_product(&H1, &P_pri, &temp_3_4);
                ulapack_transpose(&H1, &temp_4_3);
                ulapack_product(&temp_3_4, &temp_4_3, &temp_3_3);
                ulapack_add(&temp_3_3, &R1, &temp_3_3);
                ulapack_inverse(&temp_3_3, &inv_temp_3_3);

                // rt_kprintf("inv1\n");
                // ulapack_print(&inv_temp_3_3, stdout);

                ulapack_transpose(&H1, &temp_4_3);
                Matrix_t temp_temp_4_3;
                ulapack_init(&temp_temp_4_3, 4, 3);
                ulapack_product(&P_pri, &temp_4_3, &temp_temp_4_3);
                ulapack_product(&temp_temp_4_3, &inv_temp_3_3, &K);

                // rt_kprintf("K1\n");
                // ulapack_print(&K, stdout);

                ekf_mpu9250_compose_g(&q_pri);

                // q_cor = K * (accel - h)
                Matrix_t accel_normalized;
                ulapack_init(&accel_normalized, 3, 1);
                ulapack_scale(&accel, 1.0/norm_a, &accel_normalized);
                ulapack_subtract(&accel_normalized, &h1, &temp_3_1);
                ulapack_product(&K, &temp_3_1, &q_cor);
                ulapack_edit_entry(&q_cor, 3, 0, 0.0);

                // q_post = q_pri + q_cor
                ulapack_add(&q_pri, &q_cor, &q_post);

                // P_post = (P_pri - K * H * P_pri)
                ulapack_product(&K, &H1, &temp_4_4);
                // ulapack_init(&temp_temp_4_4, 4, 4);
                ulapack_product(&temp_4_4, &P_pri, &temp_temp_4_4);
                ulapack_subtract(&P_pri, &temp_temp_4_4, &P_post);


                /* update stage 2 */
                ekf_mpu9250_compose_jacobian_2(&H2, &q_pri);
                ulapack_scale(&H2, Vm.entry[1][0], &H2);
                ulapack_scale(&H1, Vm.entry[2][0], &temp_3_4);
                ulapack_add(&H2, &temp_3_4, &H2);

                // K = P_pri * H' * (H * P_pri * H' + R)^-1
                ulapack_product(&H2, &P_pri, &temp_3_4);
                ulapack_transpose(&H2, &temp_4_3);
                ulapack_product(&temp_3_4, &temp_4_3, &temp_3_3);
                ulapack_add(&temp_3_3, &R2, &temp_3_3);
                ulapack_inverse(&temp_3_3, &inv_temp_3_3);

                // rt_kprintf("inv2\n");
                // ulapack_print(&inv_temp_3_3, stdout);

                ulapack_transpose(&H2, &temp_4_3);
                ulapack_product(&P_pri, &temp_4_3, &temp_temp_4_3);
                ulapack_product(&temp_temp_4_3, &inv_temp_3_3, &K);

                // rt_kprintf("K2\n");
                // ulapack_print(&K, stdout);

                ekf_mpu9250_compose_mag(&q_pri);
                ulapack_scale(&h2, Vm.entry[1][0], &h2);
                ulapack_scale(&h1, Vm.entry[2][0], &temp_3_1);
                ulapack_add(&h2, &temp_3_1, &h2);


                // q_cor = K * (mag - h)
                Matrix_t mag_normalized;
                ulapack_init(&mag_normalized, 3, 1);
                ulapack_scale(&mag, 1.0/norm_mag, &mag_normalized);
                ulapack_subtract(&mag_normalized, &h2, &temp_3_1);
                ulapack_product(&K, &temp_3_1, &q_cor);
                ulapack_edit_entry(&q_cor, 1, 0, 0.0);
                ulapack_edit_entry(&q_cor, 2, 0, 0.0);

                // q_post = q_post + q_cor
                ulapack_add(&q_post, &q_cor, &q_post);

                // P_post = (P_post - K * H * P_post)
                ulapack_product(&K, &H2, &temp_4_4);
                ulapack_product(&temp_4_4, &P_post, &temp_temp_4_4);
                ulapack_subtract(&P_post, &temp_temp_4_4, &P_post);

                // rt_kprintf("P_post\n");
                // ulapack_print(&P_post, stdout);
                // ekf_mpu9250_init_P_post();
            }


            // normalize quaternion
            MatrixEntry_t q_post_norm;
            ulapack_norm(&q_post, &q_post_norm);
            ulapack_scale(&q_post, 1.0f / q_post_norm, &q_post);

            ekf_mpu9250_quaternion_to_euler(&q_post, &euler);

            ulapack_scale(&euler, EKF_MPU9250_RAD_TO_DEG, &euler);  // convert rad to deg

            
            if(ret == OM_OK)
            {
                // LOG_I("ax = %4d, ay = %4d, az = %4d, gx = %4d, gy = %4d, gz = %4d", mpu9250_msg_buf.accel.x, mpu9250_msg_buf.accel.y, mpu9250_msg_buf.accel.z, mpu9250_msg_buf.gyro.x, mpu9250_msg_buf.gyro.y, mpu9250_msg_buf.gyro.z, mpu9250_msg_buf.mag.x, mpu9250_msg_buf.mag.y, mpu9250_msg_buf.mag.z);
                // rt_kprintf("%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d\n", mpu9250_msg_buf.accel.x, mpu9250_msg_buf.accel.y, mpu9250_msg_buf.accel.z, mpu9250_msg_buf.gyro.x, mpu9250_msg_buf.gyro.y, mpu9250_msg_buf.gyro.z, mpu9250_msg_buf.mag.x, mpu9250_msg_buf.mag.y, mpu9250_msg_buf.mag.z);
                rt_kprintf("roll: %4d, pitch: %4d, yaw: %4d\n", (int)euler.entry[0][0], (int)euler.entry[1][0], (int)euler.entry[2][0]);

            }
            else
            {
                LOG_E("om_suber_export failed , err code: %d", ret);
            }
        }
    }


}

rt_err_t ekf_mpu9250_init(void)
{
    rt_err_t ret = RT_EOK;

    ret = rt_thread_init(&ekf_mpu9250_thread,
                         "ekf_mpu9250",
                         ekf_mpu9250_thread_entry,
                         RT_NULL,
                         &ekf_mpu9250_thread_stack[0],
                         sizeof(ekf_mpu9250_thread_stack),
                         EKF_MPU9250_PRIORITY,
                         EKF_MPU9250_TIMESLICE);
    if (ret != RT_EOK)
    {
        LOG_E("init ekf_mpu9250 thread failed");
        return ret;
    }

    rt_thread_startup(&ekf_mpu9250_thread);

    return ret;
}
