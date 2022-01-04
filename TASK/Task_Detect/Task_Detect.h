/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       detect_task.c/h
  * @brief      心跳任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
#ifndef DETECH_TASK_H
#define DETECH_TASK_H



/*OS控制任务周期以及启动时间*/
#define Detect_TASK_INIT_TIME 5
#define Detect_CONTROL_TIME   2

extern  void Detect_TASK(void const * argument);


#endif
