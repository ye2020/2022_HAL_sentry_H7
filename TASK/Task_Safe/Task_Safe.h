/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       safe_task.c/h
  * @brief      安全检测任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
	
#ifndef SAFE_TASK_H
#define SAFE_TASK_H


/*OS控制任务周期以及启动时间*/
#define SAFE_TASK_INIT_TIME 	5
#define SAFE_CONTROL_TIME_MS  2

extern  void SAFE_TASK(void const * argument);


#endif
