/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       safe_task.c/h
  * @brief      完成底盘控制任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/

#include "Task_Safe.h"
#include "SysInit.h"	

void SAFE_TASK(void const * argument)
{
			vTaskDelay(SAFE_TASK_INIT_TIME);

		while(1)
		{
		
			vTaskDelay(SAFE_CONTROL_TIME_MS); //系统延时

		}
	
	
}
