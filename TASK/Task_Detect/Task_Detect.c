/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       detect_task.c/h
  * @brief      ��������
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
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
		/* ����������� */
	vTaskDelay(Detect_CONTROL_TIME);

	}	
	
	
}
