#include "rope_task.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "string.h"
#include "vofa.h"
/*
can id��master idͬ��
��һ�ؽ�roll��pitch���idΪ1��2
�ڶ��ؽ�roll��pitch���idΪ3��4
�������Ĺؽ���can2ͨ����idΪ5��6
*/

float pos_test,vel_test;
uint16_t test_id;
arm_theta_param HEU_ARM;//������е�۽�������ṹ��
arm_basic_param ROPE_ARM;//������е�ۻ�����ʼ�������ṹ��
motor_fbpara_t armmotor[6];//�ܹ���������Ĳ����ṹ��
float motor_angle[6],motor_speed[6];//������ɺ�ֵ�������Ŀ����ֵ������Ƕ���ת�����ٶȣ���λΪrad��rad/s		
/**
  * @brief          RTOS����
  * @param[in]      none
  * @retval         none
  */
void rope_cal_process(void)
{ 	
  motor_enable_init();//���ʹ�ܳ�ʼ��
  arm_param_init(&ROPE_ARM);//��е�۲�����ʼ��
  for(;;)
	  {	
		 //pos_speed_ctrl(&hfdcan1,1, pos_test, vel_test);//����
		 //vofa_demo(); //vofa��λ��ʹ��demo
		 //vofa_user(float data1,float data2,float data3,float data4) 
		  angle_trans(&HEU_ARM,1);
		 osDelay(2);//����ִ��Ƶ�ʣ�1000/2=500hz
	  }
}
float delta_lr,delta_lp;//Ϊ�˷���ʵʱ�۲���������ȫ�֣�Ϊ����ʱ����������
/**
  * @brief          ��е�۵ĽǶȽ��㣬������λ�������������ս��㵽���ת���ĽǶ�
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
		pos_speed_ctrl(&hfdcan1,1, motor_angle[0], 5);//Ŀǰû�й켣�ٶȹ滮����ʱ���ö���ִ��
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
  * @brief          �����ֽṹ�������ʼ��
  * @param[in]      none
  * @retval         none
  */
void motor_enable_init(void)
{
	for(uint8_t j = 0; j < 6; j++)//can1���ʹ�ܳ�ʼ��
	{
		for (uint8_t i = 0; i < 10; i++)
			{
				enable_motor_mode(&hfdcan1,j,POS_MODE);
				osDelay(10);
			}
	}
  	for(uint8_t j = 0; j < 3; j++)//can2���ʹ��
	{
		for (uint8_t i = 0; i < 10; i++)
			{
				enable_motor_mode(&hfdcan2,j,POS_MODE);
				osDelay(10);
			}
	}
}
/**
  * @brief          ������е�ۻ���������ʼ����
  * @param[in]      none
  * @retval         none
  */
 void arm_param_init(arm_basic_param *ARM_PARA)
 {
	ARM_PARA->h = ARM__H;
	ARM_PARA->n = ARM__N;
	ARM_PARA->w = ARM__W;
	ARM_PARA->i_motor_arm =1.0f ;//���д�ģ�������
	ARM_PARA->d =1.0f ;
	
 }
/**
  * @brief          ������λ������
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

