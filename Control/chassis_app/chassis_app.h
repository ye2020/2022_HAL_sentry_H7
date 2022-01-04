#ifndef __CHASSIS_APP_H__
#define __CHASSIS_APP_H__

#include "main.h"	
#include "RemoteControl.h"
#include "CAN_1_Receive.h"

	
extern void chassis_app_init(void);
extern void Chassis_CAN_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
extern void Chassis_CAN2_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
extern void CAN2_Chassis_RC_SetMsg(const RC_ctrl_t *can1_RC_send);
extern void CAN2_Chassis_YAW_SetMsg(const motor_measure_t *can1_YAW_send);
extern void CAN1_Chassis_yaw_Setmsg(int16_t ESC_208);


#endif
