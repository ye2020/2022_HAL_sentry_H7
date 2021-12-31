#ifndef _FDCAN_RECEIVE_H
#define _FDCAN_RECEIVE_H

#include "main.h"

//rm电机统一数据结构体
typedef struct
{
    uint16_t position;
    int16_t speed;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_position;


    int16_t angle;
    int16_t speed_filt;
    int16_t first_Flag;
    int32_t yaw_angle;
    int32_t pitch_angle;
    int32_t actual_Position;  //真实位置
} motor_measure_t;



void fdcan1_config(void);
void fdcan2_config(void);

void Chassis_CAN_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
 void CAN1_chassis_receive(FDCAN_HandleTypeDef *hcan);
void CAN2_chassis_receive(FDCAN_HandleTypeDef *hcan);
void Chassis_CAN2_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);


#endif