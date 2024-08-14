#include "rope_task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "string.h"
#include "vofa.h"
/*
can id与master id同号
第一关节roll和pitch电机id为1与2
第二关节roll与pitch电机id为3与4
第三第四关节走can2通道，id为5与6
*/

float pos_test,vel_test;
uint16_t test_id;
arm_theta_param HEU_ARM;//绳驱机械臂解算参数结构体
arm_basic_param ROPE_ARM;//绳驱机械臂基础初始化参数结构体
motor_fbpara_t armmotor[6];//总共六个电机的参数结构体
float motor_angle[6],motor_speed[6];//解算完成后赋值给电机的目标数值，电机角度与转动的速度，单位为rad与rad/s		
/**
  * @brief          RTOS任务
  * @param[in]      none
  * @retval         none
  */
void rope_cal_process(void)
{ 	
  motor_enable_init();//电机使能初始化
  arm_param_init(&ROPE_ARM);//机械臂参数初始化
  for(;;)
	  {	
		 //pos_speed_ctrl(&hfdcan1,1, pos_test, vel_test);//测试
		 //vofa_demo(); //vofa上位机使用demo
		 //vofa_user(float data1,float data2,float data3,float data4) 
		  angle_trans(&HEU_ARM,1);
		 osDelay(2);//任务执行频率，1000/2=500hz
	  }
}
float delta_lr,delta_lp;//为了方便实时观测所以用了全局，为计算时的绳长变量
/**
  * @brief          机械臂的角度解算，根据上位机传回数据最终解算到点击转动的角度
  * @param[in]      none
  * @retval         none
  */
void angle_trans(arm_theta_param *arm_data,uint8_t joint)
{
	float roll_angle ,pitch_angle;
	roll_angle = *arm_data->roll_angel;
	pitch_angle = *arm_data->pitch_angle;

	delta_lr =ROPE_ARM.n*ROPE_ARM.w*(sin(roll_angle)*sin(pitch_angle/2));
	delta_lp = ROPE_ARM.n*ROPE_ARM.w*(cos(roll_angle)*sin(pitch_angle/2));
	switch (joint)
	{
	case 1:
		motor_angle[0] = delta_lr*2/(ROPE_ARM.d*ROPE_ARM.i_motor_arm);
		motor_angle[1] = delta_lp*2/(ROPE_ARM.d*ROPE_ARM.i_motor_arm);
		pos_speed_ctrl(&hfdcan1,1, motor_angle[0], 5);//目前没有轨迹速度规划，暂时采用定速执行
		pos_speed_ctrl(&hfdcan1,2, motor_angle[1], 5);
		break;
	case 2:
		motor_angle[2] = delta_lr*2/(ROPE_ARM.d*ROPE_ARM.i_motor_arm);
		motor_angle[3] = delta_lp*2/(ROPE_ARM.d*ROPE_ARM.i_motor_arm);
		pos_speed_ctrl(&hfdcan1,3, motor_angle[2], 5);
		pos_speed_ctrl(&hfdcan1,4, motor_angle[3], 5);
		break;
	case 3:
		motor_angle[4] = delta_lr*2/(ROPE_ARM.d*ROPE_ARM.i_motor_arm);
		motor_angle[5] = delta_lp*2/(ROPE_ARM.d*ROPE_ARM.i_motor_arm);
		pos_speed_ctrl(&hfdcan2,5, motor_angle[4], 5);
		pos_speed_ctrl(&hfdcan2,6, motor_angle[5], 5);
		break;
	default:
		break;
	}
	
}
/**
  * @brief          绳驱手结构体参数初始化
  * @param[in]      none
  * @retval         none
  */
void motor_enable_init(void)
{
	for(uint8_t j = 0; j < 6; j++)//can1电机使能初始化
	{
		for (uint8_t i = 0; i < 10; i++)
			{
				enable_motor_mode(&hfdcan1,j,POS_MODE);
				osDelay(10);
			}
	}
  	for(uint8_t j = 0; j < 3; j++)//can2电机使能
	{
		for (uint8_t i = 0; i < 10; i++)
			{
				enable_motor_mode(&hfdcan2,j,POS_MODE);
				osDelay(10);
			}
	}
}
/**
  * @brief          绳驱机械臂基础参数初始化，
  * @param[in]      none
  * @retval         none
  */
 void arm_param_init(arm_basic_param *ARM_PARA)
 {
	ARM_PARA->h = ARM__H;
	ARM_PARA->n = ARM__N;
	ARM_PARA->w = ARM__W;
	ARM_PARA->i_motor_arm =1.0f ;//随便写的，并不对
	ARM_PARA->d =1.0f ;
	
 }
/**
  * @brief          接受上位机数据
  * @param[in]      none
  * @retval         none
  */
 void computer_rev(uint8_t *buf,uint8_t length )
 {
	if(buf[0]==0x32)
	{
		memcpy(&HEU_ARM,buf,sizeof(HEU_ARM));
	}
 }

