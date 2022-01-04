/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       automatic_strike.c/h
  * @brief      自瞄部分代码
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/

#include "automatic_strike.h"
#include "SysInit.h"


/************************ 变量 **************************/

#define USART3_RX_LEN   32
#define USART3_TX_LEN   32

extern UART_HandleTypeDef huart3;                                     // 串口三句柄
extern DMA_HandleTypeDef hdma_usart3_rx;                              // DMA句柄
fifo_rx_def fifo_usart3_rx;                                           // 串口三接收环形队列
fifo_rx_def fifo_usart3_tx;

static fifo_rx_def *pfifo_visual = &fifo_usart3_rx;                   // 小电脑环形队列

static uint8_t Usart3_Tx[USART3_TX_LEN] = {0};
static uint8_t Usart3_Rx[USART3_RX_LEN] = {0};												// DMA接收缓冲区
static uint8_t Usart3_Rx_Buffer[USART3_RX_LEN] = {0};									// 接收队列
static uint8_t Usart3_Tx_Buffer[USART3_TX_LEN] = {0};

Vision_Auto_Data_t Vision_Auto_Data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};   // 储存小电脑返回的数据



/************************ 函数声明 *****************************/
float hex2Float(uint8_t HighByte, uint8_t LowByte);
uint32_t usart3_dma_send(uint8_t *data, uint16_t len);



//返回视觉自瞄控制变量，通过指针传递方式传递信息
Vision_Auto_Data_t *Get_Auto_Control_Point(void)
{
    return &Vision_Auto_Data;
}

/**
  * @brief          串口三环形队列初始化
  * @param[in]      none
  * @retval         none
  * @attention      功能：初始化环形队列 包括接收缓冲区和长度等
  */
void automatic_init(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  // 使能空闲中断
    /* 使能DMA中断 */


    /* 使能串口DMA接收 */
		HAL_UART_Receive_DMA(&huart3, Usart3_Rx, USART3_RX_LEN); // 开启串口DMA接收（记得开启循环模式）

    if (fifo_init(pfifo_visual, Usart3_Rx_Buffer, USART3_RX_LEN) == -1)
    {
        Error_Handler(); // 必须 2 的幂次方
    }

    /* 使能串口DMA发送 */
    __HAL_DMA_DISABLE(huart3.hdmatx);  // 关闭串口DMA发送通道 （不用开启循环模式）
    
    if (fifo_init(&fifo_usart3_tx, Usart3_Tx_Buffer, USART3_TX_LEN) == -1)
    {
        Error_Handler(); // 必须 2 的幂次方
    }
}


/**
  * @brief          串口三视觉中断函数
  * @param[in]      none
  * @retval         none
  * @attention      功能：视觉数据接收,保存在接收数组中
  */
void USER_UART3_IRQHandler(UART_HandleTypeDef *huart)
{

	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) == SET)          // 检测是否是空闲中断
	{		
    volatile uint32_t num = 0;

		__HAL_UART_CLEAR_IDLEFLAG(&huart3);		// 清除挂起标准（H7必须要手动清）
		num = huart->Instance->ISR;                                   //清除RXNE标志
		num = huart->Instance->RDR;                                   //清USART_IT_IDLE标志

//    __HAL_DMA_CLEAR_FLAG(huart ->hdmarx, DMA_FLAG_TCIF1_5);      // 清除传输完成标准位 
		__HAL_UART_CLEAR_PEFLAG(&huart3);															 // 清除传输完成标志位 

//		__HAL_DMA_DISABLE(&hdma_usart3_rx);
    __HAL_DMA_DISABLE(huart->hdmarx);                            // 关闭串口DMA发送通道

    num = USART3_RX_LEN - __HAL_DMA_GET_COUNTER(huart3.hdmarx);  //! 获取DMA中未传输的数据个数，NDTR寄存器分析参考中文参考手册 （DMA_Channel_TypeDef）  这个不同的芯片HAL库里面定义的命名有点不同

    fifo_write_buff(pfifo_visual ,Usart3_Rx,num);                // 写入环形队列
		
		__HAL_DMA_ENABLE(&hdma_usart3_rx);														// 开启DMA（记得呀 ！！）
    HAL_UART_Receive_DMA(&huart3,Usart3_Rx,USART3_RX_LEN);       // 启动DMA
  }

}

/**
  * @brief      发送数据给视觉
  * @param[in]  data: 敌方装甲板  红：0 | 蓝：1
  * @param[in]  auto_pitch_angle: p轴角度
  * @param[in]  shoot_speed: 射速，
  * @retval     none
  * @attention  协议：帧头0xFF  数据  帧尾0xFE
  *             通过放入数据到fifo缓冲区，
  *             然后在调用函数将缓冲区的数据放进DMA的数组里面，开启DMA传输
  */
void MiniPC_Send_Data(uint8_t data, uint8_t moauto_pitch_anglede, uint8_t shoot_speed)
{
    uint8_t SendBuff[5];

    SendBuff[0] = 0xFF;
    SendBuff[1] = data;
    SendBuff[2] = moauto_pitch_anglede;
    SendBuff[3] = shoot_speed;
    SendBuff[4] = 0xFE;

    usart3_dma_send(SendBuff,ARR_SIZE(SendBuff));
}


/*
*名称：MiniPC串口接收数据处理
*功能：通过串口三接收并保存MiniPC发来的数据
*输入：无
*输出：无
*/
   
void MiniPC_Data_Deal(void) 
{

		uint8_t buff_read[32];
	
    uint32_t length = fifo_read_buff(pfifo_visual , buff_read , ARR_SIZE(buff_read));

    if (length)
    {
        for (uint16_t i = 0; i < length; i++)
        {
          if (buff_read[i] == 0xFF && buff_read[i + 7] == 0xFE)
          { 
              Vision_Auto_Data.auto_yaw_angle   = (hex2Float(buff_read[i + 2], buff_read[i + 1]) / 100);    //自动打击的y轴角度计算/100
              Vision_Auto_Data.auto_pitch_angle = (hex2Float(buff_read[i + 4], buff_read[i + 3]) / 100);    //自动打击的p轴角度计算
              Vision_Auto_Data.len              = (hex2Float(buff_read[i + 6], buff_read[i + 5]) / 100);    // 距离

              i = i + 7;

          }
        }
    }

    else
    {
        // 没有数据
    }

        if (pfifo_visual->error)
    {
        // 接收错误
        pfifo_visual->error = 0;
    }
}


/*
*功能：高低八位数据整合
*/
float hex2Float(uint8_t HighByte, uint8_t LowByte)
{
    float high = (float) (HighByte & 0x7f);
    float low  = (float) LowByte;

    if (HighByte & 0x80)//MSB is 1 means a negative number
    {
        return (high * 256.0f + low) - 32768;
    }
    else
    {
        return (high * 256.0f + low);
    }
}



/**
  * @brief          串口三 + DMA 发送
  * @param[in]      *data
  * @param[in]      *len
  * @retval         void
  */

uint32_t usart3_dma_send(uint8_t *data, uint16_t len)
{	
	uint32_t result = fifo_write_buff(&fifo_usart3_tx, data, len); //将数据放入循环缓冲区
	
    if (result != 0 && ((huart3.gState == HAL_UART_STATE_READY)|| (huart3.gState == HAL_UART_STATE_BUSY_TX)))
    {
        len = fifo_read_buff(&fifo_usart3_tx, Usart3_Tx, USART3_TX_LEN); //从循环缓冲区获取数据
		
	  	if (HAL_UART_Transmit_DMA(&huart3, Usart3_Tx, len) != HAL_OK)
	  	{
		  	//Error_Handler();
	  	}
    }
	
	if (result == len)
	{
		return len;
	}
	else
	{
		return result;
	}
}

