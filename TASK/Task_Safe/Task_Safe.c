/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       safe_task.c/h
  * @brief      ��ɵ��̿�������
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/

#include "Task_Safe.h"
#include "SysInit.h"	

void SAFE_TASK(void const * argument)
{
			vTaskDelay(SAFE_TASK_INIT_TIME);

		while(1)
		{
		
			vTaskDelay(SAFE_CONTROL_TIME_MS); //ϵͳ��ʱ

		}
	
	
}
