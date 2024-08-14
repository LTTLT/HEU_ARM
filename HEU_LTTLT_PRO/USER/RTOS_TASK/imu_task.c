#include "imu_task.h"
#include "cmsis_os.h"
#include "string.h"//memcpy报错时记得引用
imu_Eular_angle eluar_angle;
imu_w_speed w_speed;
q_wxzy quaternion;
/*
陀螺仪id通过can2回传数据，id设置为？
每个数据包数据位dlc为6，四元数为8
其中角速度和欧拉角，四元数为0x280+id,0x380+id,0x480+id
单位为°/s
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

