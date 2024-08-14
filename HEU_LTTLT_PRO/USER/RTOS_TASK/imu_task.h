#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "stdint.h"


#define IMU_CAN_ID 8
#define IMU_EULAR_DATA_LENGTH 6
#define IMU_W_SPEED_DATA_LENGTH 6
#define Q4_LENGTH 8


#pragma pack (1)//傻逼keil的错位问题，会导致结构地址对不齐
typedef struct
{
int16_t roll;
int16_t pitch;
int16_t yaw;
} imu_Eular_angle;//陀螺仪can回传欧拉角数据结构体

typedef struct
{
    int16_t x_w;
    int16_t y_w;
    int16_t z_w;
} imu_w_speed;//陀螺仪can回传角速度数据结构体

typedef struct
{
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} q_wxzy;//陀螺仪can回传四元数数据结构体
#pragma pack()

enum imu_data_model//枚举结构体，节省代码量
{
      EULAR=1, W_SPD, Q4
};

extern void imu_data_process(uint8_t model,uint8_t *databuf);

#endif
