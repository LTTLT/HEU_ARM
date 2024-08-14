#include "imu_task.h"
#include "cmsis_os.h"
#include "string.h"//memcpy����ʱ�ǵ�����
imu_Eular_angle eluar_angle;
imu_w_speed w_speed;
q_wxzy quaternion;
/*
������idͨ��can2�ش����ݣ�id����Ϊ��
ÿ�����ݰ�����λdlcΪ6����Ԫ��Ϊ8
���н��ٶȺ�ŷ���ǣ���Ԫ��Ϊ0x280+id,0x380+id,0x480+id
��λΪ��/s
*/

void imu_updata()
{   
    

}

void imu_data_process(uint8_t model,uint8_t *databuf)
{
    switch (model)
    {
    case EULAR:
        memcpy(&eluar_angle,databuf,IMU_EULAR_DATA_LENGTH);
        break;
    case W_SPD:
        memcpy(&w_speed,databuf,IMU_W_SPEED_DATA_LENGTH);
        break;
    case Q4:
        memcpy(&quaternion,databuf,Q4_LENGTH);
        break;
    default:
        break;
    }

}

