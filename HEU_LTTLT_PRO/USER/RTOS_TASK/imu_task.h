#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "stdint.h"


#define IMU_CAN_ID 8
#define IMU_EULAR_DATA_LENGTH 6
#define IMU_W_SPEED_DATA_LENGTH 6
#define Q4_LENGTH 8


#pragma pack (1)//ɵ��keil�Ĵ�λ���⣬�ᵼ�½ṹ��ַ�Բ���
typedef struct
{
int16_t roll;
int16_t pitch;
int16_t yaw;
} imu_Eular_angle;//������can�ش�ŷ�������ݽṹ��

typedef struct
{
    int16_t x_w;
    int16_t y_w;
    int16_t z_w;
} imu_w_speed;//������can�ش����ٶ����ݽṹ��

typedef struct
{
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} q_wxzy;//������can�ش���Ԫ�����ݽṹ��
#pragma pack()

enum imu_data_model//ö�ٽṹ�壬��ʡ������
{
      EULAR=1, W_SPD, Q4
};

extern void imu_data_process(uint8_t model,uint8_t *databuf);

#endif
