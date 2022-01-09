/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			can_1_receive.c/h
 *
 * @brief 		����ң������ʼ����ң�������ݻ�ȡ��ң����ͨѶЭ��Ľ���
 *
 * @note  		ң�������ݽ��ղ��ô��ڼ�DMA��ģʽ
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 */

#ifndef CAN_1_RECEIVE_H__
#define CAN_1_RECEIVE_H__


#include "main.h"

#define gimbal_yaw_TO_chassis     2 //�ɰ汾can2 ������can2��������̰壬���̰��������yaw����    1 -> can1����   2 -> can2����

/* CAN����ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_M3508_MOTOR1_ID = 0x201,
    CAN_M3508_MOTOR2_ID = 0x202,
    CAN_M3508_MOTOR3_ID = 0x203,
    CAN_M3508_MOTOR4_ID = 0x204,

    CAN_TRIGGER_MOTORA_ID = 0x205,
    CAN_TRIGGER_MOTORB_ID = 0x206,
    CAN_PIT_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
	
    CAN_ABOVEGIMBAL_YAW_MOTOR_ID = 0x208,
    /* ����̨���ִ����can2�����ô�ö�١� */
		#if (gimbal_yaw_TO_chassis == 2)
	    CAN_YAW_MOTOR_ID = 0x208,

		#elif(gimbal_yaw_TO_chassis == 1)
		CAN_YAW_MOTOR_ID = 0x5ff,

		#endif

    CAN_ENEMY_COLOR_ID = 0x3ff,
} can_msg_id_e;


//rm���ͳһ���ݽṹ��
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
    int32_t actual_Position;  //��ʵλ��
} motor_measure_t;




void CAN1_chassis_receive(FDCAN_HandleTypeDef *hcan);
void CAN1_filter_config(void);
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan);
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//extern void (*CAN1_receive_callback)(FDCAN_HandleTypeDef *);

#if (gimbal_yaw_TO_chassis == 1)
// ����yaw����ָ��
motor_measure_t *Chassis_Get_Yaw_Gimbal_Motor_Measure_Point(void);
#endif
const motor_measure_t *Get_Fire_MotorB_Measure_Point(void);
const motor_measure_t *Get_Fire_MotorA_Measure_Point(void);
motor_measure_t *Get_Pitch_Gimbal_Motor_Measure_Point(void);
extern void (*CAN1_receive_callback)(FDCAN_HandleTypeDef *);
extern void (*CAN2_receive_callback)(FDCAN_HandleTypeDef *);
extern void chassis_can1_callback (FDCAN_HandleTypeDef *hcan);
extern void gimbal_can1_callback (FDCAN_HandleTypeDef *hcan);
extern motor_measure_t *Get_Pitch_AboveGimbal_Motor_Measure_Point(void);
extern const motor_measure_t *Get_Yaw_AboveGimbal_Motor_Measure_Point(void);


#endif

