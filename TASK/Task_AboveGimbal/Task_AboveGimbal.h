/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       Task_AboveGimbal.c/h
  * @brief      ����̨����
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/
#include "Task_Gimbal.h"


#ifndef TASK_ABOVEGIMBAL_H
#define TASK_ABOVEGIMBAL_H

#define AboveGimbal_TASK_INIT_TIME 5    
#define AboveGimbal_CONTROL_TIME   2


/******************** pitch PID ���� **********************/
#define ABOVE_GIMBAL_P_PITCH_P  60.0f         //Pλ�û�  70
#define ABOVE_GIMBAL_P_PITCH_I 	1.5f		// 1.1f
#define ABOVE_GIMBAL_P_PITCH_D  75.0f         //0.0f   75

#define ABOVE_GIMBAL_S_PITCH_P  10.5f     //P�ٶȻ�(��Ҫ��i) 1.5     9.5
#define ABOVE_GIMBAL_S_PITCH_I  0.0f
#define ABOVE_GIMBAL_S_PITCH_D  6.0f	    //6.0f    //3.5  4

/******************** ����pitch PID ���� **********************/
#define ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_P 36.0f    //P�Զ�λ�û�  30.0f |530.0f |35 |  30
#define ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_I 0.1f
#define ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_D 9.0f     //200.0f    9.0

#define ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_P 10.0f     //P�Զ��ٶȻ�  12  8
#define ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f
#define ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      //5

/******************** yaw PID ���� **********************/
#define ABOVE_GIMBAL_P_YAW_P  130.0f   //Yλ�û�    150     62    130
#define ABOVE_GIMBAL_P_YAW_I  0.0f
#define ABOVE_GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define ABOVE_GIMBAL_S_YAW_P  9.0f     //Y�ٶȻ�     8      10     9
#define ABOVE_GIMBAL_S_YAW_I  0.0f
#define ABOVE_GIMBAL_S_YAW_D  0.0f     //2                   0

/******************** ����yaw PID ���� **********************/
#define ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_P 100.0f      //Y�Զ�λ�û� 63.0f |60.0f |170
#define ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_I 0.0f
#define ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_D 15.0f

#define ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_P 15.0f       //Y�Զ��ٶȻ� 12
#define ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f
#define ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_D 1.0f


extern void AboveGimbal_TASK(void const * argument);
extern VisionStatus_E  get_Enemy_status_from_above(void);


#endif

