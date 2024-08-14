#ifndef ROPE_TASK_H
#define ROPE_TASK_H
#include "pid.h"
#include "dm4310_drv.h"
#define ARM__N 2.0f//��Ȧ����
#define ARM__W 100.0f//������ˮƽ����
#define ARM__H 150.0f//�����뾶*2
#define ROPE_D 1.0F;//�����ӵĻ���ֱ��
#define PI 3.14159265f

#pragma pack (1)//ɵ��keil�Ĵ�λ���⣬�ᵼ�½ṹ��ַ�Բ���
typedef struct
{
	uint8_t header;
    float  roll_angel[3];//��z����ת����
	float  pitch_angle[3];//�����Ƕ�
//	float  rope_length[3][2];
} arm_theta_param;
#pragma pack()
typedef struct
{
    float  n;//�����Ƶ�Ȧ��
	float  w; //����������֮��ľ���
    float  h;//����ƽ����ϵ�����뾶*2
    float  i_motor_arm;//�����ȣ����ת��һȦ�ı��������
    float  d;//���������������ӵĻ��ֵ�ֱ��
} arm_basic_param;

extern motor_fbpara_t armmotor[6];

void rope_cal_process(void);
void motor_enable_init(void);
void computer_rev(uint8_t *buf,uint8_t length );
void arm_param_init(arm_basic_param *ARM_PARA);
void angle_trans(arm_theta_param *arm_data,uint8_t joint);
#endif
