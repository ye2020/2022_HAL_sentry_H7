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
	
#include "Task_LED.h"
#include "SysInit.h"	

void Led_Task(void const * argument)
{
	    //空闲一段时间
    vTaskDelay(LED_TASK_INIT_TIME);	
	
	while(1)
	{
			
		LEDE2(1);
		LEDE3(1);
		LEDE4(1);
		LEDE5(1);
		
							//检测周期
		vTaskDelay(LED_CONTROL_TIME);
			
	}
	
}

