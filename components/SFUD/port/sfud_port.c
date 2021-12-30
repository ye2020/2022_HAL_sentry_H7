/* - (SFUD移植接口
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2018, zylx, <qgyhd1234@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2018-11-23
 */

#include <sfud.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "spi.h"
#include "sfud_flash_def.h"
#include "main.h"
//#include "shell_port.h"


/*--------------------变量-----------------------*/
extern SPI_HandleTypeDef hspi2;

typedef struct
{
    SPI_HandleTypeDef *spix;								// 外围接口
    GPIO_TypeDef *cs_gpiox;						
    uint16_t cs_gpio_pin;
} spi_user_data, *spi_user_data_t;

static char log_buf[256];

static spi_user_data nor_spi1 = {.spix = &hspi2, .cs_gpiox = FLASH_CS_GPIO_PORT, .cs_gpio_pin = FLASH_CS_PIN };

/*--------------------函数-----------------------*/
void sfud_log_debug(const char *file, const long line, const char *format, ...);
void sfud_log_info(const char *format, ...);


/**
  * @brief      spi_lock 函数 （锁）
  * @param[in]  *spi: sfud_spi结构体
  * @retval     none
  * @attention  用户需要自己实现
  */
static void spi_lock(const sfud_spi *spi)
{
    __disable_irq();
}

/**
  * @brief      spi_unlock 函数 （锁）
  * @param[in]  *spi: sfud_spi结构体
  * @retval     none
  * @attention  用户需要自己实现
  */
static void spi_unlock(const sfud_spi *spi)
{
    __enable_irq();
}

///**
//  * @brief      SPI 读写
//  * @param[in]  *spi: sfud_spi结构体
//  * @param[in]  *write_buf: 需要写的数据
//  * @param[in]  write_size: 写数据大小
//  * @param[in]  *read_buf: 需要读的数据
//  * @param[in]  read_size: 读数据大小
//  * @retval     sfud_err类型
//  * @attention  用户需要自己实现SPI的读和写
//  */

//static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf, size_t read_size)
//{
//    sfud_err result = SFUD_SUCCESS;
//    uint8_t send_data, read_data;

//    spi_user_data_t spi_dev = (spi_user_data_t)spi->user_data;

//    if (write_size)
//    {
//        SFUD_ASSERT(write_buf);
//    }
//    if (read_size)
//    {
//        SFUD_ASSERT(read_buf);
//    }

//    /* reset cs pin */
//    if (spi_dev->cs_gpiox != NULL)
//        SPI_FLASH_CS_LOW();

//    do
//    {
//        /* 开始读写数据 */
//        for (size_t i = 0, retry_times; i < write_size + read_size; i++)
//        {
//            /* 先写缓冲区中的数据到 SPI 总线，数据写完后，再写 dummy(0xFF) 到 SPI 总线 */
//            if (i < write_size)
//            {
//                send_data = *write_buf++;
//            }
//            else
//            {
//                send_data = SFUD_DUMMY_DATA;
//            }

//            /* 发送数据 */
//            retry_times = 1000;
//						
//						/* 在接收缓冲区为空时，等待 。           while (SPI_I2S_GetFlagStatus(spi_dev->spix, SPI_FLAG_RXNE) == RESET)*/
//            while (__HAL_SPI_GET_FLAG(spi_dev -> hspilx, SPI_FLAG_RXP) == RESET)
//            {
//                SFUD_RETRY_PROCESS(NULL, retry_times, result);
//            }
//            if (result != SFUD_SUCCESS)
//            {
//                break;
//            }
////            SPI_I2S_SendData(spi_dev->spix, send_data); //库函数
//							HAL_SPI_Transmit(spi_dev ->hspilx, &send_data, read_size, retry_times); //HAL库
//            //WRITE_REG(spi_dev->spix->DR, send_data); //库函数寄存器

//            /* 接收数据 */
//            retry_times = 1000;
//            while (__HAL_SPI_GET_FLAG(spi_dev -> hspilx, SPI_FLAG_RXP) == RESET)
//            {
//                SFUD_RETRY_PROCESS(NULL, retry_times, result);
//            }
//            if (result != SFUD_SUCCESS)
//            {
//                break;
//            }
////            read_data = SPI_I2S_ReceiveData(spi_dev->spix); //库函数
//            read_data = READ_REG(spi_dev->hspilx ->Instance ->RXDR); //hal库函数寄存器 （RXDR ：spi接收数据寄存器）

//            /* 写缓冲区中的数据发完后，再读取 SPI 总线中的数据到读缓冲区 */
//            if (i >= write_size)
//            {
//                *read_buf++ = read_data;
//            }
//        }
//    } while (0);

//    /* set cs pin */
//    if (spi_dev->cs_gpiox != NULL)
//		SPI_FLASH_CS_HIGH();

//    return result;
//}

///**
//  * @brief      SPI 读写
//  * @param[in]  *spi: sfud_spi结构体
//  * @param[in]  *write_buf: 需要写的数据
//  * @param[in]  write_size: 写数据大小
//  * @param[in]  *read_buf: 需要读的数据
//  * @param[in]  read_size: 读数据大小
//  * @retval     sfud_err类型
//  * @attention  用户需要自己实现SPI的读和写
//  */

static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf,
        size_t read_size) {
    sfud_err result = SFUD_SUCCESS;
    spi_user_data_t spi_dev = (spi_user_data_t) spi->user_data;
    /**
     * add your spi write and read code
     */
//    RT_ASSERT(spi);
    if (write_size)
    {
        SFUD_ASSERT(write_buf);
    }
    if (read_size)
    {
        SFUD_ASSERT(read_buf);
    }

			 HAL_GPIO_WritePin(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin, GPIO_PIN_RESET);
    if(write_size && read_size)
{
        if(HAL_SPI_Transmit(spi_dev->spix, (uint8_t *)write_buf, write_size, 1000)!=HAL_OK) 
        {
            result = SFUD_ERR_WRITE;
        }   
       /* 为了简单起见，这个例子只是等待传输，但应用程序可能在传输操作时执行其他任务正在进行 */
        while (HAL_SPI_GetState(spi_dev->spix) != HAL_SPI_STATE_READY);
        if(HAL_SPI_Receive(spi_dev->spix, (uint8_t *)read_buf, read_size, 1000)!=HAL_OK)
        {
            result = SFUD_ERR_READ;
        }
    }else if(write_size)
{
        if(HAL_SPI_Transmit(spi_dev->spix, (uint8_t *)write_buf, write_size, 1000)!=HAL_OK) 
        {
            result = SFUD_ERR_WRITE;
        }
    }else
{
        if(HAL_SPI_Receive(spi_dev->spix, (uint8_t *)read_buf, read_size, 1000)!=HAL_OK)
        {
            result = SFUD_ERR_READ;
        }
}
       /* For simplicity reasons, this example is just waiting till the end of the
    transfer, but application may perform other tasks while transfer operation
    is ongoing. */
    while (HAL_SPI_GetState(spi_dev->spix) != HAL_SPI_STATE_READY);
		 HAL_GPIO_WritePin(spi_dev->cs_gpiox, spi_dev->cs_gpio_pin, GPIO_PIN_SET);
    return result;
}

#ifdef SFUD_USING_QSPI //使能QSPI通讯模式
/**
 * 通过QSPI读取flash数据
 */
static sfud_err qspi_read(const struct __sfud_spi *spi, uint32_t addr, sfud_qspi_read_cmd_format *qspi_read_cmd_format, uint8_t *read_buf, size_t read_size)
{

    sfud_err result = SFUD_SUCCESS;
    QSPI_CommandTypeDef Cmdhandler;
    extern QSPI_HandleTypeDef hqspi;

    /* set cmd struct */
    Cmdhandler.Instruction = qspi_read_cmd_format->instruction;
    if (qspi_read_cmd_format->instruction_lines == 0)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_NONE;
    }
    else if (qspi_read_cmd_format->instruction_lines == 1)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    }
    else if (qspi_read_cmd_format->instruction_lines == 2)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_2_LINES;
    }
    else if (qspi_read_cmd_format->instruction_lines == 4)
    {
        Cmdhandler.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    }

    Cmdhandler.Address = addr;
    Cmdhandler.AddressSize = QSPI_ADDRESS_24_BITS;
    if (qspi_read_cmd_format->address_lines == 0)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_NONE;
    }
    else if (qspi_read_cmd_format->address_lines == 1)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_1_LINE;
    }
    else if (qspi_read_cmd_format->address_lines == 2)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_2_LINES;
    }
    else if (qspi_read_cmd_format->address_lines == 4)
    {
        Cmdhandler.AddressMode = QSPI_ADDRESS_4_LINES;
    }

    Cmdhandler.AlternateBytes = 0;
    Cmdhandler.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    Cmdhandler.AlternateBytesSize = 0;

    Cmdhandler.DummyCycles = qspi_read_cmd_format->dummy_cycles;

    Cmdhandler.NbData = read_size;
    if (qspi_read_cmd_format->data_lines == 0)
    {
        Cmdhandler.DataMode = QSPI_DATA_NONE;
    }
    else if (qspi_read_cmd_format->data_lines == 1)
    {
        Cmdhandler.DataMode = QSPI_DATA_1_LINE;
    }
    else if (qspi_read_cmd_format->data_lines == 2)
    {
        Cmdhandler.DataMode = QSPI_DATA_2_LINES;
    }
    else if (qspi_read_cmd_format->data_lines == 4)
    {
        Cmdhandler.DataMode = QSPI_DATA_4_LINES;
    }

    Cmdhandler.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
    Cmdhandler.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    Cmdhandler.DdrMode = QSPI_DDR_MODE_DISABLE;
    Cmdhandler.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    HAL_QSPI_Command(&hqspi, &Cmdhandler, 5000);

    if (HAL_QSPI_Receive(&hqspi, read_buf, 5000) != HAL_OK)
    {
        sfud_log_info("qspi recv data failed(%d)!", hqspi.ErrorCode);
        hqspi.State = HAL_QSPI_STATE_READY;
        result = SFUD_ERR_READ;
    }

    return result;
}
#endif /* SFUD_USING_QSPI */

/* about 100 microsecond delay */
static void retry_delay_100us(void)
{
    uint32_t delay = 120;
    while (delay--)
        ;
}

sfud_err sfud_spi_port_init(sfud_flash *flash)
{
    sfud_err result = SFUD_SUCCESS;

    /**
     * add your port spi bus and device object initialize code like this:
     * 1. rcc initialize
     * 2. gpio initialize
     * 3. spi device initialize
     * 4. flash->spi and flash->retry item initialize
     *    flash->spi.wr = spi_write_read; //Required
     *    flash->spi.qspi_read = qspi_read; //Required when QSPI mode enable
     *    flash->spi.lock = spi_lock;
     *    flash->spi.unlock = spi_unlock;
     *    flash->spi.user_data = &spix;
     *    flash->retry.delay = null;
     *    flash->retry.times = 10000; //Required
     */
    switch (flash->index)
    {
    //与sfud_cfg.h中的配对
    case SFUD_W25Q32JVSSIQ_DEVICE_INDEX:
    {
        /* 1.RCC、2.GPIO、3.SPI外设初始化 */
				MX_SPI2_Init();
		
        /* 4.设置接口和数据 */
        flash->spi.wr = spi_write_read;
        //flash->spi.qspi_read = qspi_read;
        flash->spi.lock = spi_lock;
        flash->spi.unlock = spi_unlock;
        flash->spi.user_data = &nor_spi1;
        /* 大约100微秒延迟 */
        flash->retry.delay = retry_delay_100us;
        /* 大约60秒超时 */
        flash->retry.times = 60 * 10000;

        break;
    }
    }

    return result;
}

/**
 * 这个函数是打印调试信息。
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 */
void sfud_log_debug(const char *file, const long line, const char *format, ...)
{
//    va_list args;

//    /* args point to the first variable parameter */
//    va_start(args, format);
//    printf("[SFUD](%s:%ld) ", file, line);
//    /* must use vprintf to print */
//    vsnprintf(log_buf, sizeof(log_buf), format, args);
//    printf("%s\n", log_buf);
//    shellWriteEndLine(&shell, ("\r\n"), sizeof("\r\n"));
//    va_end(args);
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD](%s:%ld) ", file, line);
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\r\n", log_buf);
    va_end(args);
}

/**
 * 这个函数是打印例程信息。
 *
 * @param format output format
 * @param ... args
 */
void sfud_log_info(const char *format, ...)
{
//    extern Shell shell;
//    va_list args;

//    /* args point to the first variable parameter */
//    va_start(args, format);
//    printf("[SFUD]");
//    //shellWriteEndLine(&shell, ("[SFUD]"), sizeof("[SFUD]"));
//    /* must use vprintf to print */
//    vsnprintf(log_buf, sizeof(log_buf), format, args);
//    printf("%s\n", log_buf);
//    //shellWriteEndLine(&shell, log_buf, sizeof(log_buf));
//    shellWriteEndLine(&shell, ("\r\n"), sizeof("\r\n"));
//    va_end(args);
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD]");
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\r\n", log_buf);
    va_end(args);
}

///**
// * SFUD演示的第一个闪存设备测试。
// *
// * @param  addr: flash的起始地址
// * @param  size: 测试flash大小
// * @param  *data: 测试flash数据缓冲区
// */
//#define SFUD_DEMO_TEST_BUFFER_SIZE 1024
////static uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE];  //sfud测试缓冲区
////sfud_demo(0, sizeof(sfud_demo_test_buf), sfud_demo_test_buf);
//int sfud_demo(uint32_t addr) //输入 - 0
//{
//    uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE]; //sfud测试缓冲区
//    size_t size = sizeof(sfud_demo_test_buf);
//    uint8_t *data = sfud_demo_test_buf;

//    sfud_err result = SFUD_SUCCESS;
//    /* sfud_get_device_table()
//	 * 获取 Flash 设备对象在 SFUD 配置文件中会定义 Flash 设备表，
//	 * 负责存放所有将要使用的 Flash 设备对象，
//	 * 所以 SFUD 支持多个 Flash 设备同时驱动。
//	 * 本方法通过 Flash 设备位于设备表中索引值来返回 Flash 设备对象，
//	 * 超出设备表范围返回 NULL 。
//	 */
//    const sfud_flash *flash = sfud_get_device_table() + 0;
//    size_t i;

//    /* 准备写数据 */
//    for (i = 0; i < size; i++)
//    {
//        data[i] = i;
//    }

//    /* 删除测试  ： 擦除闪存数据*/
//    result = sfud_erase(flash, addr, size);
//    if (result == SFUD_SUCCESS)
//    {
//        //printf("Erase the %s flash data finish. Start from 0x%08X, size is %d.\r\n", flash->name, addr, size);
//        printf("擦除%s闪存数据完成。 从0x%08X开始，大小为%d。\r\n", flash->name, addr, size);
//    }
//    else
//    {
//        //printf("Erase the %s flash data failed.\r\n", flash->name);
//        printf("擦除%s闪存数据失败。\r\n", flash->name);
//        return 0;
//    }

//    /* 写测试 */
//    result = sfud_write(flash, addr, size, data);
//    if (result == SFUD_SUCCESS)
//    {
//        //printf("Write the %s flash data finish. Start from 0x%08X, size is %d.\r\n", flash->name, addr, size);
//        printf("写入%s flash数据完成。从0x%08X开始，大小为%d。\r\n", flash->name, addr, size);
//    }
//    else
//    {
//        //printf("Write the %s flash data failed.\r\n", flash->name);
//        printf("写入%s flash数据失败。\r\n", flash->name);
//        return 0;
//    }

//    /* 读测试 */
//    result = sfud_read(flash, addr, size, data);
//    if (result == SFUD_SUCCESS)
//    {
//        //printf("Read the %s flash data success. Start from 0x%08X, size is %d. The data is:\r\n", flash->name, addr, size);
//        //printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
//        printf("读取%s flash data成功。从0x%08X开始，大小为%d。的数据是:\r\n", flash->name, addr, size);
//        printf("偏移量(h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
//        for (i = 0; i < size; i++)
//        {
//            if (i % 16 == 0)
//            {
//                printf("[%08X] ", addr + i);
//            }
//            printf("%02X ", data[i]);
//            if (((i + 1) % 16 == 0) || i == size - 1)
//            {
//                printf("\r\n");
//            }
//        }
//        printf("\r\n");
//    }
//    else
//    {
//        //printf("Read the %s flash data failed.\r\n", flash->name);
//        printf("读取%s闪存数据失败。\r\n", flash->name);
//    }

//    /* 数据检查 */
//    for (i = 0; i < size; i++)
//    {
//        if (data[i] != i % 256)
//        {
//            //printf("Read and check write data has an error. Write the %s flash data failed.\r\n", flash->name);
//            printf("读取和检查写入数据错误。写入%s flash数据失败。\r\n", flash->name);
//            break;
//        }
//    }
//    if (i == size)
//    {
//        //printf("The %s flash test is success.\r\n", flash->name);
//        printf("%s flash测试成功。\r\n", flash->name);
//    }

//    return 0;
//}
////!这里用了shell
////SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), sfud_demo, sfud_demo, sfud_demo); //导出到命令列表里
