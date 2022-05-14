/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			bsp_dwt.c/h
 *
 * @brief 		STM32利用bwt进行延时
 *
 * @note  		利用BWT实现在RTOS和HAL实现微秒级延时	
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */
#include "bsp_dwt.h"
/*
* 1. 先使能DWT外设，这个由另外内核调试寄存器DEMCR的位24控制，写1使能
* 2. 使能CYCCNT寄存器之前，先清0。
* 3. 使能CYCCNT寄存器，这个由DWT的CYCCNTENA 控制，也就是DWT控制寄存器的位0控制，写1使 能
*/

//寄存器基地址
#define 	DWT_CR 			 *(uint32_t*)0xE0001000
#define   DWT_CYCCNT   *(uint32_t*)0xE0001004
#define		DEM_CR 			 *(uint32_t*)0xE000EDFC

//定义需使能位
#define		DEM_CR_TRCENA				(1<<24)
#define		DWT_CR_CYCCNTENA		(1<<0)

uint32_t CPU_FREQ_Hz;			//系统时钟 单位 MHz
static uint32_t CYCCNT_RountCount;
static float DWT_Timeline;


void DWT_reset_tick(void)
{
	//系统时钟 单位 MHz
	CPU_FREQ_Hz = HAL_RCC_GetHCLKFreq() / (1000 * 1000);	//GET_CPU_ClkFreq()	HAL_RCC_GetHCLKFreq()

	return; 
}

void DWT_init(void) 
{
	/* 使能DWT外设 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	
	/* DWT CYCCNT寄存器计数清0 */
	DWT->CYCCNT = (uint32_t) 0u;
	
	/* 使能Cortex-M DWT CYCCNT寄存器 */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	//HAL_RCC_GetHCLKFreq(); //16000000UL
	DWT_reset_tick();
	CYCCNT_RountCount = 0;
	DWT_Timeline = 0; 
}


/*
* 主频168MHz的情况下，32位计数器计满是2^32/168000000 = 25.565秒
* 		建议使用本函数做延迟的话，延迟在1秒以下。
* 两个32位无符号数相减，获取的结果再赋值给32位无符号数依然可以正确的获取差值。
* 		假如A,B,C都是32位无符号数。
* 		如果A > B 那么A - B = C，这个很好理解，完全没有问题
* 		 如果A < B 那么A - B = C， C的数值就是0xFFFFFFFF - B + A + 1。这一点要特别注
意，正好用于本函数。
*/
void DWT_delay_us(uint32_t us) 
{
	uint32_t tStart;
	
	us *= CPU_FREQ_Hz; // 需要延时的节拍数
	tStart = DWT->CYCCNT;
	
	//两个无符号的数相减，如果前者比后者小，会发生什么问题？这样是否就达不到准确延时的目的了？是否需要考虑溢出的情况？
	while ((DWT->CYCCNT - tStart) < us); /* 求减过程中，如果发生第一次32位计数器重新计数，依然可以正确计算 */
}


void DWT_delay_ms(uint32_t ms)
{
	DWT_delay_us(1000 * ms);
}

uint32_t DWT_get_time(void)
{
	return (DWT->CYCCNT);
}


//测试代码运行时间（加在被测代码块之后）
float DTW_Time_Difference_ms(void)
{
  static uint32_t old_counter;
  uint32_t counter,couter_current;
  couter_current = DWT_get_time();
  if(couter_current > old_counter)
    counter = couter_current - old_counter;
  else
    counter = couter_current + 0XFFFFFFFF - old_counter;
  old_counter = couter_current;
  return (counter / (SystemCoreClock/1000/2));
}