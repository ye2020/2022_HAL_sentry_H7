/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       Task_LED.c/h
  * @brief      LED��������
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/
	
#include "Task_LED.h"
#include "SysInit.h"	

void Led_Task(void const * argument)
{
	    //����һ��ʱ��
    vTaskDelay(LED_TASK_INIT_TIME);	
	
	while(1)
	{
			
		LEDE2(1);
		LEDE3(1);
		LEDE4(1);
		LEDE5(1);
		
							//�������
		vTaskDelay(LED_CONTROL_TIME);
			
	}
	
}

