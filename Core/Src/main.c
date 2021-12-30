/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sfud.h"
#include "stdio.h"
#include "FDCAN_Receive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 定义变量
sfud_flash  sfud_norflash0 = {.name = "flash0", \
															.spi.name = "SPI0",\
															.chip = {"W25Q32BV", SFUD_MF_ID_WINBOND, 0x40, 0x16, 4L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20}
															};
extern UART_HandleTypeDef huart1;


int sfud_demo(uint32_t addr, size_t size, uint8_t *data); //输入 - 0

#define SFUD_DEMO_TEST_BUFFER_SIZE                     1024
static uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE];

//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//PUTCHAR_PROTOTYPE
//{
//    //具体哪个串口可以更改USART1为其它串口
//		HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1 , 0xffff);
//    return ch;
//}
//int fputc(int ch,FILE *f)
//{
//    uint8_t temp[1]={ch};
//    HAL_UART_Transmit(&huart1,temp,1,2);        //UartHandle是串口的句柄
////		return ch;
////}
//// 重定向函数1
//#ifdef __GNUC__
///* With GCC, small printf (option LD Linker->Libraries->Small printf
//   set to 'Yes') calls __io_putchar() */
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
// 
///**
//  * @brief  Retargets the C library printf function to the USART.
//  * @param  None
//  * @retval None
//  */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//  return ch;
//}
// 
//int _write(int32_t file, uint8_t *ptr, int32_t len)
//{
//	/* Implement your write code here, this is used by puts and printf for example */
//	int DataIdx;
//	for (DataIdx = 0; DataIdx < len; DataIdx++)
//	{
//		__io_putchar(*ptr++);
//	}
//	return len;
//	/* return len; */
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief      拨码开关数值检测   
  * @param[in]  none
  * @retval     none
  * @attention  
  */
static  uint16_t  DIP_Switch(void)
{
	uint16_t value;
	
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

uint16_t a=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
//	printf("\r\n 1\r\n");
		a = DIP_Switch();
		
	/* SFUD初始化 */
//	if(sfud_init() != SFUD_SUCCESS)
//	{
//	 printf("SFUD init fail.\r\n");
//	}

//if (sfud_device_init(&sfud_norflash0) == SFUD_SUCCESS)
//{				
//	
//        /* enable qspi fast read mode, set one data lines width */
//        
//        sfud_demo(0, sizeof(sfud_demo_test_buf), sfud_demo_test_buf);
//}

	fdcan1_config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			
				
//	sfud_init();
//		sfud_demo(0);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Chassis_CAN_Send_Msg(5000,0,0,0);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 24;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#  if 0
void sfud_demo(uint32_t addr, size_t size, uint8_t *data)
{
    sfud_err result = SFUD_SUCCESS;
    extern sfud_flash *sfud_dev;
    const sfud_flash *flash = &sfud_norflash0;//sfud_get_device(SFUD_W25Q32JVSSIQ_DEVICE_INDEX);
    size_t i;
    /* prepare write data */
    for (i = 0; i < size; i++)
    {
        data[i] = i;
    }
    /* 删除测试 */
    result = sfud_erase(flash, addr, size);
    if (result == SFUD_SUCCESS)
    {
        printf("Erase the %s flash data finish. Start from 0x%08X, size is %zu.\r\n", flash->name, addr, size);
    }
    else
    {
        printf("Erase the %s flash data failed.\r\n", flash->name);
        return;
    }
    /* 写测试 */
    result = sfud_write(flash, addr, size, data);
    if (result == SFUD_SUCCESS)
    {
        printf("Write the %s flash data finish. Start from 0x%08X, size is %zu.\r\n", flash->name, addr, size);
    }
    else
    {
        printf("Write the %s flash data failed.\r\n", flash->name);
        return;
    }
    /* read test */
    result = sfud_read(flash, addr, size, data);
    if (result == SFUD_SUCCESS)
    {
        printf("Read the %s flash data success. Start from 0x%08X, size is %zu. The data is:\r\n", flash->name, addr, size);
        printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        for (i = 0; i < size; i++)
        {
            if (i % 16 == 0)
            {
                printf("[%08X] ", addr + i);
            }
            printf("%02X ", data[i]);
            if (((i + 1) % 16 == 0) || i == size - 1)
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
    }
    else
    {
        printf("Read the %s flash data failed.\r\n", flash->name);
    }
    /* data check */
    for (i = 0; i < size; i++)
    {

        if (data[i] != i % 256)
        {
            printf("Read and check write data has an error. Write the %s flash data failed.\r\n", flash->name);
            break;
        }
    }
    if (i == size)
    {
        printf("The %s flash test is success.\r\n", flash->name);
    }
}

#endif
#if 1
///**
// * SFUD演示的第一个闪存设备测试。
// *
// * @param  addr: flash的起始地址
// * @param  size: 测试flash大小
// * @param  *data: 测试flash数据缓冲区
// */
int sfud_demo(uint32_t addr, size_t size, uint8_t *data) //输入 - 0
{
    sfud_err result = SFUD_SUCCESS;
    extern sfud_flash *sfud_dev;
    const sfud_flash *flash = &sfud_norflash0;//sfud_get_device(SFUD_W25Q32JVSSIQ_DEVICE_INDEX);

    /* sfud_get_device_table()
	 * 获取 Flash 设备对象在 SFUD 配置文件中会定义 Flash 设备表，
	 * 负责存放所有将要使用的 Flash 设备对象，
	 * 所以 SFUD 支持多个 Flash 设备同时驱动。
	 * 本方法通过 Flash 设备位于设备表中索引值来返回 Flash 设备对象，
	 * 超出设备表范围返回 NULL 。
	 */
//    const sfud_flash *flash = sfud_get_device_table() + 0;
    size_t i;

    /* 准备写数据 */
    for (i = 0; i < size; i++)
    {
        data[i] = i;
    }

    /* 删除测试  ： 擦除闪存数据*/
    result = sfud_erase(flash, addr, size);
    if (result == SFUD_SUCCESS)
    {
        //printf("Erase the %s flash data finish. Start from 0x%08X, size is %d.\r\n", flash->name, addr, size);
        printf("擦除%s闪存数据完成。 从0x%08X开始，大小为%d。\r\n", flash->name, addr, size);
    }
    else
    {
        //printf("Erase the %s flash data failed.\r\n", flash->name);
        printf("擦除%s闪存数据失败。\r\n", flash->name);
        return 0;
    }

    /* 写测试 */
    result = sfud_write(flash, addr, size, data);
    if (result == SFUD_SUCCESS)
    {
        //printf("Write the %s flash data finish. Start from 0x%08X, size is %d.\r\n", flash->name, addr, size);
        printf("写入%s flash数据完成。从0x%08X开始，大小为%d。\r\n", flash->name, addr, size);
    }
    else
    {
        //printf("Write the %s flash data failed.\r\n", flash->name);
        printf("写入%s flash数据失败。\r\n", flash->name);
        return 0;
    }

    /* 读测试 */
    result = sfud_read(flash, addr, size, data);
    if (result == SFUD_SUCCESS)
    {
        //printf("Read the %s flash data success. Start from 0x%08X, size is %d. The data is:\r\n", flash->name, addr, size);
        //printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        printf("读取%s flash data成功。从0x%08X开始，大小为%d。的数据是:\r\n", flash->name, addr, size);
        printf("偏移量(h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        for (i = 0; i < size; i++)
        {
            if (i % 16 == 0)
            {
                printf("[%08X] ", addr + i);
            }
            printf("%02X ", data[i]);
            if (((i + 1) % 16 == 0) || i == size - 1)
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
    }
    else
    {
        //printf("Read the %s flash data failed.\r\n", flash->name);
        printf("读取%s闪存数据失败。\r\n", flash->name);
    }

    /* 数据检查 */
    for (i = 0; i < size; i++)
    {
        if (data[i] != i % 256)
        {
            //printf("Read and check write data has an error. Write the %s flash data failed.\r\n", flash->name);
            printf("读取和检查写入数据错误。写入%s flash数据失败。\r\n", flash->name);
            break;
        }
    }
    if (i == size)
    {
        //printf("The %s flash test is success.\r\n", flash->name);
        printf("%s flash测试成功。\r\n", flash->name);
    }

    return 0;
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
