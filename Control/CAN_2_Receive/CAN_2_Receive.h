/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			can_2_receive.c/h
 *
 * @brief 	can2滤波器初始化以及底盘云台，can2接收
 *
 * @note  		
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */

#ifndef CAN_2_RECEIVE_H__
#define CAN_2_RECEIVE_H__

#include "main.h"
#include "CAN_1_Receive.h"
#include "Task_Gimbal.h"



extern void CAN2_filter_config(void);
void CAN2_chassis_receive(FDCAN_HandleTypeDef *hcan);

// 返回yaw轴电机指针
const motor_measure_t *Get_Yaw_Gimbal_Motor_Measure_Point(void);
#if (gimbal_yaw_TO_chassis == 1)
int16_t Get_Yaw_Gimbal_Motor_Output(void);
#endif

extern void chassis_can2_callback (FDCAN_HandleTypeDef *hcan);
extern void gimbal_can2_callback (FDCAN_HandleTypeDef *hcan);
extern VisionStatus_E  get_Enemy_status(void);



#endif
