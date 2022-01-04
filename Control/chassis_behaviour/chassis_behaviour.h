/**
  ******************************************************************************
  * @file       chassis_behaviour.c/h
  * @brief      µ×ÅÌ×´Ì¬»ú¡£
  * @history    2021.11.18
  ******************************************************************************
  */

 #ifndef  __CHASSIS_BEHAVIOUR_H
 #define __CHASSIS_BEHAVIOUR_H

#define RC_SW1_lift   3
#define RC_SW2_right  3

#include "Task_Chassis.h"

extern void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f);


#endif
