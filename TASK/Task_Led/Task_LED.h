/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       Task_LED.c/h
  * @brief      LED心跳任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
	
#ifndef TASK_LED_H
#define TASK_LED_H
	
	
	
/*OS控制任务周期以及启动时间*/
#define LED_TASK_INIT_TIME 2
#define LED_CONTROL_TIME   2	

extern void Led_Task(void const * argument);

	
#endif
