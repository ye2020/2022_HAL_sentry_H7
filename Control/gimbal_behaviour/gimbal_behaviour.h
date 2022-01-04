/**
******************************************************************************
* @file       gimbal_behaviour.c/h
* @brief      ??????
******************************************************************************
*/


#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "Task_Gimbal.h"
#include "Task_Fire.h"


#define ENEMY_DISAPPEAR_TIMES  400		//检测到敌人消失的次数

#define PITCH_UP   0
#define PITCH_DOWN 1

#define RIGHT   0
#define LEFT 1

extern void Gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f);
extern void Gimbal_Stop(gimbal_control_t *gimbal_stop_f);
extern Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void);
extern Fire_WorkStatus_e Return_Fire_Mode(void);




#endif
