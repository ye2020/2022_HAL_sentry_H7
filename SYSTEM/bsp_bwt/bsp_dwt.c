/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			bsp_dwt.c/h
 *
 * @brief 		STM32����bwt������ʱ
 *
 * @note  		����BWTʵ����RTOS��HALʵ��΢�뼶��ʱ	
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 */
#include "bsp_dwt.h"
/*
* 1. ��ʹ��DWT���裬����������ں˵��ԼĴ���DEMCR��λ24���ƣ�д1ʹ��
* 2. ʹ��CYCCNT�Ĵ���֮ǰ������0��
* 3. ʹ��CYCCNT�Ĵ����������DWT��CYCCNTENA ���ƣ�Ҳ����DWT���ƼĴ�����λ0���ƣ�д1ʹ ��
*/

//�Ĵ�������ַ
#define 	DWT_CR 			 *(uint32_t*)0xE0001000
#define   DWT_CYCCNT   *(uint32_t*)0xE0001004
#define		DEM_CR 			 *(uint32_t*)0xE000EDFC

//������ʹ��λ
#define		DEM_CR_TRCENA				(1<<24)
#define		DWT_CR_CYCCNTENA		(1<<0)

uint32_t CPU_FREQ_Hz;			//ϵͳʱ�� ��λ MHz
static uint32_t CYCCNT_RountCount;
static float DWT_Timeline;


void DWT_reset_tick(void)
{
	//ϵͳʱ�� ��λ MHz
	CPU_FREQ_Hz = HAL_RCC_GetHCLKFreq() / (1000 * 1000);	//GET_CPU_ClkFreq()	HAL_RCC_GetHCLKFreq()

	return; 
}

void DWT_init(void) 
{
	/* ʹ��DWT���� */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	
	/* DWT CYCCNT�Ĵ���������0 */
	DWT->CYCCNT = (uint32_t) 0u;
	
	/* ʹ��Cortex-M DWT CYCCNT�Ĵ��� */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	//HAL_RCC_GetHCLKFreq(); //16000000UL
	DWT_reset_tick();
	CYCCNT_RountCount = 0;
	DWT_Timeline = 0; 
}


/*
* ��Ƶ168MHz������£�32λ������������2^32/168000000 = 25.565��
* 		����ʹ�ñ��������ӳٵĻ����ӳ���1�����¡�
* ����32λ�޷������������ȡ�Ľ���ٸ�ֵ��32λ�޷�������Ȼ������ȷ�Ļ�ȡ��ֵ��
* 		����A,B,C����32λ�޷�������
* 		���A > B ��ôA - B = C������ܺ���⣬��ȫû������
* 		 ���A < B ��ôA - B = C�� C����ֵ����0xFFFFFFFF - B + A + 1����һ��Ҫ�ر�ע
�⣬�������ڱ�������
*/
void DWT_delay_us(uint32_t us) 
{
	uint32_t tStart;
	
	us *= CPU_FREQ_Hz; // ��Ҫ��ʱ�Ľ�����
	tStart = DWT->CYCCNT;
	
	//�����޷��ŵ�����������ǰ�߱Ⱥ���С���ᷢ��ʲô���⣿�����Ƿ�ʹﲻ��׼ȷ��ʱ��Ŀ���ˣ��Ƿ���Ҫ��������������
	while ((DWT->CYCCNT - tStart) < us); /* ��������У����������һ��32λ���������¼�������Ȼ������ȷ���� */
}


void DWT_delay_ms(uint32_t ms)
{
	DWT_delay_us(1000 * ms);
}

uint32_t DWT_get_time(void)
{
	return (DWT->CYCCNT);
}


//���Դ�������ʱ�䣨���ڱ�������֮��
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