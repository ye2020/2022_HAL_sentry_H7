#ifndef _RMMOTOR_H
#define _RMMOTOR_H

#include "main.h"
#include "pid.h"
#include "CAN_1_Receive.h"



/*=====函数声明========*/
void Motor_Actual_Position(motor_measure_t *rmMotor, int16_t gear_Ratio,int16_t lap_encoder); //计算真实码盘值
int16_t Angle_Limiting_Int16(int16_t Angl_Err, int16_t lap_encoder);                           //临角处理16位
int32_t Angle_Limiting_Int32(int32_t Angl_Error,int16_t buff,int16_t lap_encoder);            //临角处理32位带减速比计算
int32_t Check_CodeValue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);               //过临界值复位码盘值
int16_t Check_Motor_Block(int16_t position);                                                   //检测电机堵转
float Get_Yaw_Different_Angle(const motor_measure_t *yaw_position , int16_t Ratio);                  //获取云台与底盘差角
int16_t Encoder_Real(int32_t read_code);                                                       //多圈绝对值编码器数据转换
int16_t Yaw_Actual_Code_Conversion(int16_t actual_code , int16_t max_code , int16_t middle_code);//Yaw轴真实位置（多圈编码器）（左正右负）

int16_t Rmmotor_Speed_control(PidTypeDef *spid, int16_t setSpeed, int16_t actualSpeed, int16_t current_limit);  //电机速度闭环
int16_t Motor_Position_Speed_Control(PidTypeDef *speed_pid, PidTypeDef *position_pid, int16_t actual_position , int16_t actual_speed , int16_t setPosition, int16_t current_limit);
int16_t motor_position_Stepping(PidTypeDef *speed_pid, PidTypeDef *position_pid, int16_t actual_position, int16_t actual_speed, int16_t setPosition, int16_t current_limit);


#endif
