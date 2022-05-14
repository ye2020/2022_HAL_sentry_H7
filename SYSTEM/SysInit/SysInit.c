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
extern void OLED_ChassisDisplay(void);
extern void OLED_GimbalDisplay(void);
void OLED_SentryDisplay(void);



/************** ���� ***************/

/**
  * @brief      ȫ�ֳ�ʼ������   
  * @param[in]  none
  * @retval     none
  * @attention  
  */ 
void System_init(void)
{
		/*  DWT��ʱ��ʼ�� */
		DWT_init();
	
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
	
		/* OLED��ʼ�� */
		IIC_Init();
		OLED_Write_Init();
		OLED_Write_On();
	
		/* ������̨��ʼ��ѡ�� */
		task_init();
		
		/*  OLED��ʾ */
		OLED_SentryDisplay();

		
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
		/* OLED��ʾ */
		OLED_ChassisDisplay();
		return 1;
	}
	
	else if (app == GIMBAL_APP)
	{
		/* ��̨���ʼ�� */
		gimbal_app_init();
		/* OLED��ʾ */
		OLED_GimbalDisplay();
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


/**
  * @brief      ���� OLED ��ʾ   
  * @param[in]  none
  * @retval     none
  * @attention  
  */
void OLED_ChassisDisplay(void)
{
//		OLED_HorizontalDisplay();
		OLED_DisplayString(0,0,16,16, (const uint8_t *)"Sentry Chassis");
//		OledScrollStart();
}


/**
  * @brief      ��̨ OLED ��ʾ   
  * @param[in]  none
  * @retval     none
  * @attention  
  */
void OLED_GimbalDisplay(void)
{
//		OLED_HorizontalDisplay();
		OLED_DisplayString(0,0,16,16, (const uint8_t *)"Sentry Gimbal");
//		OledScrollStart();
}


/**
  * @brief      OLED ��ʾ   
  * @param[in]  none
  * @retval     none
  * @attention  
  */
void OLED_SentryDisplay(void)
{
		if (chassis_using)
		OLED_DisplayString(0,2,16,16, (const uint8_t *)"���� ��");
		else
		OLED_DisplayString(0,2,16,16, (const uint8_t *)"���� ��");

		if(yaw_angle_limit)
		OLED_DisplayString(64,2,16,16,(const uint8_t *)"Yaw�� ��");
		else
		OLED_DisplayString(64,2,16,16,(const uint8_t *)"Yaw�� ��");
		
		if(Debug_mode)
		OLED_DisplayString(0,4,16,16,(const uint8_t *)"���� ��");
		else
		OLED_DisplayString(0,4,16,16,(const uint8_t *)"���� ��");
	
		if(SFUD_NORFLASH)
		OLED_DisplayString(64,4,16,16,(const uint8_t *)"flash ��");
		else
		OLED_DisplayString(64,4,16,16,(const uint8_t *)"flash ��");
	
		if(hot_limit)	
		OLED_DisplayString(0,6,16,16,(const uint8_t *)"�������� ��");
		else
		OLED_DisplayString(0,6,16,16,(const uint8_t *)"�������� ��");

}
