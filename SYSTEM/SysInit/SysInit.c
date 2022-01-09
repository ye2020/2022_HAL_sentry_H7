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


/************** 函数 ***************/

/**
  * @brief      全局初始化函数   
  * @param[in]  none
  * @retval     none
  * @attention  
  */ 
void System_init(void)
{
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
	
		/* 底盘云台初始化选择 */
		task_init();
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
		return 1;
	}
	
	else if (app == GIMBAL_APP)
	{
		/* 云台板初始化 */
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

//生成[min,max]范围的随机数
int RNG_Get_RandomRange(int min,int max)
{
	uint32_t random;
	HAL_RNG_GenerateRandomNumber(&hrng,&random);
	
	return random%(max-min+1) +min;
}
