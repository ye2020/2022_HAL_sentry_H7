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
	
#ifndef TASK_LED_H
#define TASK_LED_H
	
	
	
/*OS�������������Լ�����ʱ��*/
#define LED_TASK_INIT_TIME 2
#define LED_CONTROL_TIME   2	

extern void Led_Task(void const * argument);

	
#endif
