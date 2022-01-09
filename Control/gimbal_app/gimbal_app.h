#ifndef _GIMBAL_APP_H
#define _GIMBAL_APP_H

#include "main.h"


void gimbal_app_init(void);
extern void CAN1_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208);
extern void CAN2_yaw_Setmsg(int16_t ESC_208);
extern void CAN2_Gimbal_yaw_Setmsg(int16_t ESC_208);
extern void CAN2_Enemy_status(int16_t ESC_208);



#endif
