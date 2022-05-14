/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       SysInit.c/h
  * @brief      初始化
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
	
#include "SysInit.h"

/************ 变量 **************/

extern RNG_HandleTypeDef hrng;

/* flash 测试缓冲区*/
#if SFUD_NORFLASH
#define SFUD_DEMO_TEST_BUFFER_SIZE                     1024
static uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE];
#endif
/************ 声明 **************/

static  uint16_t  DIP_Switch(void);					// 拨码开关解码
static uint8_t task_init(void);							// 底盘云台选择初始化
static uint8_t app = 0;														// 初始化状态


extern int sfud_demo(uint32_t addr, size_t size, uint8_t *data); // flash测试Demo
extern void OLED_ChassisDisplay(void);
extern void OLED_GimbalDisplay(void);
void OLED_SentryDisplay(void);



/************** 函数 ***************/

/**
  * @brief      全局初始化函数   
  * @param[in]  none
  * @retval     none
  * @attention  
  */ 
void System_init(void)
{
		/*  DWT延时初始化 */
		DWT_init();
	
		/*  can滤波配置初始化 */
		CAN1_filter_config();
	
		/*  can2滤波配置初始化 */
		CAN2_filter_config();
	
		/* 串口二环形队列初始化 */
		bsp_usart2_init();

		/* 	小电脑初始化 */
		automatic_init();
			
		/* 遥控初始化 */
		remote_control_init();
	
		/* OLED初始化 */
		IIC_Init();
		OLED_Write_Init();
		OLED_Write_On();
	
		/* 底盘云台初始化选择 */
		task_init();
		
		/*  OLED显示 */
		OLED_SentryDisplay();

		
		/* SFUD初始化 */
	#if SFUD_NORFLASH	
		
	if (sfud_init() == SFUD_SUCCESS)
	{			
			sfud_demo(0, sizeof(sfud_demo_test_buf), sfud_demo_test_buf);
	}
#endif
}


/**
  * @brief      拨码开关数值检测   
  * @param[in]  none
  * @retval     none
  * @attention  
  */                                                                                                                                                         
static  uint16_t  DIP_Switch(void)
{
	uint16_t value = 0;
	
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12) == 1)  value |= 0x01;			//最0位置1
	else																					value |= 0x00;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13) == 1)  value |= 0x02;			//最1位置1
	else																					value |= 0x00;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14) == 1)  value |= 0x04;			//最3位置1
	else																					value |= 0x00;
	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15) == 1)  value |= 0x08;			//最4位置1
	else																					value |= 0x00;
	
	return value;
}

/**
  * @brief      选择初始化 chassis_app \ gimbal_app   
  * @param[in]  none
  * @retval     none
  * @attention  
  */
static uint8_t task_init(void)
{

	app = DIP_Switch();
	
	if(app == CHASSIS_APP)
	{
		/*  底盘板初始化 */
		chassis_app_init();
		/* OLED显示 */
		OLED_ChassisDisplay();
		return 1;
	}
	
	else if (app == GIMBAL_APP)
	{
		/* 云台板初始化 */
		gimbal_app_init();
		/* OLED显示 */
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

//生成[min,max]范围的随机数
int RNG_Get_RandomRange(int min,int max)
{
	uint32_t random;
	HAL_RNG_GenerateRandomNumber(&hrng,&random);
	
	return random%(max-min+1) +min;
}


/**
  * @brief      底盘 OLED 显示   
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
  * @brief      云台 OLED 显示   
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
  * @brief      OLED 显示   
  * @param[in]  none
  * @retval     none
  * @attention  
  */
void OLED_SentryDisplay(void)
{
		if (chassis_using)
		OLED_DisplayString(0,2,16,16, (const uint8_t *)"底盘 √");
		else
		OLED_DisplayString(0,2,16,16, (const uint8_t *)"底盘 □");

		if(yaw_angle_limit)
		OLED_DisplayString(64,2,16,16,(const uint8_t *)"Yaw轴 √");
		else
		OLED_DisplayString(64,2,16,16,(const uint8_t *)"Yaw轴 □");
		
		if(Debug_mode)
		OLED_DisplayString(0,4,16,16,(const uint8_t *)"调试 √");
		else
		OLED_DisplayString(0,4,16,16,(const uint8_t *)"调试 □");
	
		if(SFUD_NORFLASH)
		OLED_DisplayString(64,4,16,16,(const uint8_t *)"flash √");
		else
		OLED_DisplayString(64,4,16,16,(const uint8_t *)"flash □");
	
		if(hot_limit)	
		OLED_DisplayString(0,6,16,16,(const uint8_t *)"热量限制 √");
		else
		OLED_DisplayString(0,6,16,16,(const uint8_t *)"热量限制 □");

}
