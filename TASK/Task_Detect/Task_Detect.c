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
	
#include "Task_Detect.h"
#include "SysInit.h"


int rng = 0;

	 
void Detect_TASK(void const * argument)
{
	vTaskDelay(Detect_TASK_INIT_TIME);
	while(1)
	{

		rng = RNG_Get_RandomRange(0,10);
		/* 心跳检测任务 */
	vTaskDelay(Detect_CONTROL_TIME);

	}	
	
	
}
