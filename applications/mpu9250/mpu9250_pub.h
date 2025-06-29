#ifndef _MPU9250_PUB_H_
#define _MPU9250_PUB_H_


#include "mpu6xxx.h"


struct mpu9250_msg_tag
{
    struct mpu6xxx_3axes accel;
    struct mpu6xxx_3axes gyro;
};

typedef struct mpu9250_msg_tag mpu9250_msg;


#endif