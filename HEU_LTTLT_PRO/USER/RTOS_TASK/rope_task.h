#ifndef ROPE_TASK_H
#define ROPE_TASK_H
#include "pid.h"
#include "dm4310_drv.h"
#define ARM__N 2.0f//绕圈次数
#define ARM__W 100.0f//滑轮组水平距离
#define ARM__H 150.0f//拟合球半径*2
#define ROPE_D 1.0F;//绕绳子的滑轮直径
#define PI 3.14159265f

#pragma pack (1)//傻逼keil的错位问题，会导致结构地址对不齐
typedef struct
{
	uint8_t header;
    float  roll_angel[3];//绕z轴旋转方向
	float  pitch_angle[3];//弯曲角度
//	float  rope_length[3][2];
} arm_theta_param;
#pragma pack()
typedef struct
{
    float  n;//绳子绕的圈数
	float  w; //两个滑轮组之间的距离
    float  h;//弯曲平面拟合的球面半径*2
    float  i_motor_arm;//传动比，电机转动一圈改变的绳长量
    float  d;//滑轮组最终绕绳子的滑轮的直径
} arm_basic_param;

extern motor_fbpara_t armmotor[6];

void rope_cal_process(void);
void motor_enable_init(void);
void computer_rev(uint8_t *buf,uint8_t length );
void arm_param_init(arm_basic_param *ARM_PARA);
void angle_trans(arm_theta_param *arm_data,uint8_t joint);
#endif
