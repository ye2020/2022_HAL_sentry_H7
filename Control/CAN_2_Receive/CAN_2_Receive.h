/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			can_2_receive.c/h
 *
 * @brief 	can2�˲�����ʼ���Լ�������̨��can2����
 *
 * @note  		
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 */

#ifndef CAN_2_RECEIVE_H__
#define CAN_2_RECEIVE_H__

#include "main.h"
#include "CAN_1_Receive.h"

extern void CAN2_filter_config(void);
void CAN2_chassis_receive(FDCAN_HandleTypeDef *hcan);

// ����yaw����ָ��
const motor_measure_t *Get_Yaw_Gimbal_Motor_Measure_Point(void);
#if (gimbal_yaw_TO_chassis == 1)
int16_t Get_Yaw_Gimbal_Motor_Output(void);
#endif

#endif
