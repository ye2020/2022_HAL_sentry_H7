/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			remote_control.c
 *
 * @brief 		包括遥控器初始化，遥控器数据获取，遥控器通讯协议的解析
 *
 * @note  		遥控器数据接收采用串口加DMA的模式
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */

#include "RemoteControl.h"
#include "main.h"
#include "SysInit.h"
#include "string.h"

/************************** 变量 ***************************/
//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700
#define USART1_RX_LEN 256
#define USART1_TX_LEN 32

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

fifo_rx_def fifo_usart1_rx;    								  // 串口三接收环形队列
fifo_rx_def fifo_usart1_tx;

fifo_rx_def *pfifo_remote = &fifo_usart1_rx;    // 遥控环形队列

RC_ctrl_t rc_ctrl;															//遥控器控制变量static

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];	//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界

//static uint8_t Usart1_Tx[USART1_TX_LEN] = {0};	//接收区
 uint8_t Usart1_Rx[32] = {0};		

 uint8_t Usart1_Rx_Buffer[USART1_RX_LEN] = {0};	//缓冲队列
//static uint8_t Usart1_Tx_Buffer[USART1_TX_LEN] = {0};


/************************** 声明 ***************************/
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

static int16_t RC_abs(int16_t value);//取正函数

/**
  * @brief          遥控任务
  * @param[in]      none
  * @retval         none
  */
void Remote_Task(void const * argument)
{

		vTaskDelay(5);
		uint8_t remote_ReadBuff[128];

	while(1)
	{
		taskENTER_CRITICAL(); //进入临界区

		LEDE5(0);																	//指示灯
		
#if (!Debug_mode)		
		if(Get_appinit_status() == CHASSIS_APP)
#endif		
		{
			fifo_read_buff(pfifo_remote,remote_ReadBuff,32);
			sbus_to_rc(remote_ReadBuff,&rc_ctrl);
		}
		 taskEXIT_CRITICAL(); //退出临界区

		vTaskDelay(2);
	}

	
}


/**
  * @brief      遥控器数据死区限制
  * @param[in]  input
  * @param[in]  dealine
  * @retval     
*/
int16_t rc_deadline_limit(int16_t input, int16_t dealine)
{
    if (input > dealine || input < -dealine)
    {
        return input;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
		hdma_usart1_rx.Instance = DMA1_Stream2;		// 定义hdma_usart1_rx.Instance为DMA1_Stream2
	
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
	
//		SCB_DisableDCache();
		

}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }

    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }

    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }

    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
//    rc_ctrl.rc.s[0] = RC_SW_DOWN;
//    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}
           
//遥控器掉线或数据错误时从启遥控器
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}



//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
		
		rc_ctrl->rc.ch[0] = rc_deadline_limit(rc_ctrl->rc.ch[0], 15); //死区限制
    rc_ctrl->rc.ch[1] = rc_deadline_limit(rc_ctrl->rc.ch[1], 15); //死区限制
    rc_ctrl->rc.ch[2] = rc_deadline_limit(rc_ctrl->rc.ch[2], 15); //死区限制
    rc_ctrl->rc.ch[3] = rc_deadline_limit(rc_ctrl->rc.ch[3], 15); //死区限制
		
		

}

/**
  * @brief          遥控器初始化函数（DMA双缓冲）
  * @param[in]      rx1_buf: 缓冲区1指针
  * @param[in]      rx2_buf: 缓冲区2指针
  * @param[in]      dma_buf_num: 缓冲区数据大小
  * @param[out]     none
  * @retval         none
  */
#if double_buffer
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart1.Instance ->CR3, USART_CR3_DMAR);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //设置DMA传输，讲串口1的数据搬运到recvive_buff中
    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], 36 );

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

	
	/* ((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->CR : H7的改动，要先定义hdma_usart1_rx.Instance = DMA1_Stream2; */
    while(DMA1_Stream2->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    DMA1_Stream2->PAR = (uint32_t) & (USART1->RDR);
 
    //内存缓冲区1
		DMA1_Stream2->M0AR = (uint32_t)(rx1_buf);

    //内存缓冲区2
		DMA1_Stream2->M1AR = (uint32_t)(rx2_buf);

    //数据长度
		DMA1_Stream2->NDTR = dma_buf_num;

    //使能双缓冲区
     SET_BIT(DMA1_Stream2->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

#endif




/**
  * @brief          遥控器初始化函数（无DMA）
  * @param[in]      none: 传入参数不需要管
  * @param[out]     none
  * @retval         none
  */
#if (!double_buffer)
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //使能DMA串口接收
    SET_BIT(huart1.Instance ->CR3, USART_CR3_DMAR);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //设置DMA传输，讲串口1的数据搬运到recvive_buff中
//    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[1], USART1_RX_LEN );
			HAL_UART_Receive_IT(&huart1, Usart1_Rx, USART1_RX_LEN);

	    if (fifo_init(&fifo_usart1_rx, Usart1_Rx_Buffer, USART1_RX_LEN) == -1)
    {
        Error_Handler(); // 必须 2 的幂次方
    }
	
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

}
#endif
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart1);
}


/**
  * @brief          串口一接收处理函数（无DMA）
  * @param[in]      huart: 串口一句柄
  * @param[out]     none
  * @retval         none
  */
#if (!double_buffer)
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	   volatile uint32_t num = 0;
	    HAL_StatusTypeDef huart_state;			// 记录串口接收状态
        
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE) == RESET)	//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET )	// 空闲中断
    {
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		}
		
//			HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[1], 32 );
		huart_state =	HAL_UART_Receive_IT(&huart1, Usart1_Rx, 32 );//用串口接收遥控数据

		fifo_write_buff(&fifo_usart1_rx, Usart1_Rx, 32);				// 写入环形队列	
}

#endif



/**
  * @brief          串口一接收处理函数（DMA双缓冲）
  * @param[in]      huart: 串口一句柄
  * @param[out]     none
  * @retval         （不知道为什么H7加了三极管反相后，DMA接收一直是零（能进中断，数据是0））
  */
#if double_buffer
// 串口中断
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	
		LEDE5(0);
	
//	__HAL_UART_CLEAR_IDLEFLAG(huart);		// 清除挂起标准
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE) == RESET)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET )
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_IDLEFLAG(huart);

       // if (( ((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->CR & DMA_SxCR_CT) == RESET)
					if((DMA1_Stream2 ->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

							SCB_InvalidateDCache_by_Addr (sbus_rx_buf[0],SBUS_RX_BUF_NUM);
            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM -  ( __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));//((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
							DMA1_Stream2->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
							DMA1_Stream2->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            SCB_InvalidateDCache_by_Addr (sbus_rx_buf[1],SBUS_RX_BUF_NUM);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM -  ( __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));//((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
             DMA1_Stream2->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
							DMA1_Stream2 ->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);

            }
        }
    }

}	

#endif






//重启遥控器
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    ((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);

}


/**
  * @brief          摇杆量清零
  * @param[in]      none
  * @retval         none
  * @attention      
  */
void Remote_reload(void)
{
	rc_ctrl.rc.ch[0] = 0;
	rc_ctrl.rc.ch[1] = 0;
	rc_ctrl.rc.ch[2] = 0;
	rc_ctrl.rc.ch[3] = 0;
	rc_ctrl.rc.ch[4] = 0;
//	rc_ctrl.rc.s1 = RC_SW_ERROR;  //出现错误为0
//	rc_ctrl.rc.s2 = RC_SW_ERROR;  //出现错误为0
	rc_ctrl.mouse.x = 0;
	rc_ctrl.mouse.y = 0;
	rc_ctrl.mouse.z = 0;
	rc_ctrl.mouse.press_l = 0;
	rc_ctrl.mouse.press_r = 0;
	rc_ctrl.key.v = 0;
}
