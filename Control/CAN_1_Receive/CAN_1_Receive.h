/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			can_1_receive.c/h
 *
 * @brief 		包括遥控器初始化，遥控器数据获取，遥控器通讯协议的解析
 *
 * @note  		遥控器数据接收采用串口加DMA的模式
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */

#ifndef CAN_1_RECEIVE_H__
#define CAN_1_RECEIVE_H__


#include "main.h"

#define gimbal_yaw_TO_chassis     2 //旧版本can2 数据由can2传输给底盘板，底盘板再输出给yaw轴电机    1 -> can1发送   2 -> can2发送

/* CAN接收ID */
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
    /* 在云台部分代码的can2处调用此枚举。 */
		#if (gimbal_yaw_TO_chassis == 2)
	    CAN_YAW_MOTOR_ID = 0x208,

		#elif(gimbal_yaw_TO_chassis == 1)
		CAN_YAW_MOTOR_ID = 0x5ff,

		#endif

    CAN_ENEMY_COLOR_ID = 0x3ff,
} can_msg_id_e;


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




void CAN1_chassis_receive(FDCAN_HandleTypeDef *hcan);
void CAN1_filter_config(void);
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan);
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//extern void (*CAN1_receive_callback)(FDCAN_HandleTypeDef *);

#if (gimbal_yaw_TO_chassis == 1)
// 返回yaw轴电机指针
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

