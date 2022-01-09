/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       SysInit.c/h
  * @brief      ��ʼ��
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/
	
#include "SysInit.h"

/************ ���� **************/

extern RNG_HandleTypeDef hrng;

/* flash ���Ի�����*/
#if SFUD_NORFLASH
#define SFUD_DEMO_TEST_BUFFER_SIZE                     1024
static uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE];
#endif
/************ ���� **************/

static  uint16_t  DIP_Switch(void);					// ���뿪�ؽ���
static uint8_t task_init(void);							// ������̨ѡ���ʼ��
static uint8_t app = 0;														// ��ʼ��״̬


extern int sfud_demo(uint32_t addr, size_t size, uint8_t *data); // flash����Demo


/************** ���� ***************/

/**
  * @brief      ȫ�ֳ�ʼ������   
  * @param[in]  none
  * @retval     none
  * @attention  
  */ 
void System_init(void)
{
		/*  can�˲����ó�ʼ�� */
		CAN1_filter_config();
	
		/*  can2�˲����ó�ʼ�� */
		CAN2_filter_config();
	
		/* ���ڶ����ζ��г�ʼ�� */
		bsp_usart2_init();

		/* 	С���Գ�ʼ�� */
		automatic_init();
			
		/* ң�س�ʼ�� */
		remote_control_init();
	
		/* ������̨��ʼ��ѡ�� */
		task_init();
		/* SFUD��ʼ�� */
	#if SFUD_NORFLASH	
		
	if (sfud_init() == SFUD_SUCCESS)
	{			
			sfud_demo(0, sizeof(sfud_demo_test_buf), sfud_demo_test_buf);
	}
#endif
}


/**
  * @brief      ���뿪����ֵ���   
  * @param[in]  none
  * @retval     none
  * @attention  
  */                                                                                                                                                         
static  uint16_t  DIP_Switch(void)
{
	uint16_t value = 0;
	
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12) == 1)  value |= 0x01;			//��0λ��1
	else																					value |= 0x00;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13) == 1)  value |= 0x02;			//��1λ��1
	else																					value |= 0x00;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14) == 1)  value |= 0x04;			//��3λ��1
	else																					value |= 0x00;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15) == 1)  value |= 0x08;			//��4λ��1
	else																					value |= 0x00;
	
	return value;
}

/**
  * @brief      ѡ���ʼ�� chassis_app \ gimbal_app   
  * @param[in]  none
  * @retval     none
  * @attention  
  */
static uint8_t task_init(void)
{

	app = DIP_Switch();
	
	if(app == CHASSIS_APP)
	{
		/*  ���̰��ʼ�� */
		chassis_app_init();
		return 1;
	}
	
	else if (app == GIMBAL_APP)
	{
		/* ��̨���ʼ�� */
		gimbal_app_init();
		return 1;
	}

	else
	{
		return 0;
	}
}



uint8_t Get_appinit_status(void)
{
	return app;
}	

//����[min,max]��Χ�������
int RNG_Get_RandomRange(int min,int max)
{
	uint32_t random;
	HAL_RNG_GenerateRandomNumber(&hrng,&random);
	
	return random%(max-min+1) +min;
}
