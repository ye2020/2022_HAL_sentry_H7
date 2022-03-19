/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			remote_control.c
 *
 * @brief 		����ң������ʼ����ң�������ݻ�ȡ��ң����ͨѶЭ��Ľ���
 *
 * @note  		ң�������ݽ��ղ��ô��ڼ�DMA��ģʽ
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 */

#include "RemoteControl.h"
#include "main.h"
#include "SysInit.h"
#include "string.h"

/************************** ���� ***************************/
//ң����������������
#define RC_CHANNAL_ERROR_VALUE 700
#define USART1_RX_LEN 256
#define USART1_TX_LEN 32

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

fifo_rx_def fifo_usart1_rx;    								  // ���������ջ��ζ���
fifo_rx_def fifo_usart1_tx;

fifo_rx_def *pfifo_remote = &fifo_usart1_rx;    // ң�ػ��ζ���

RC_ctrl_t rc_ctrl;															//ң�������Ʊ���static

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];	//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��

//static uint8_t Usart1_Tx[USART1_TX_LEN] = {0};	//������
 uint8_t Usart1_Rx[32] = {0};		

 uint8_t Usart1_Rx_Buffer[USART1_RX_LEN] = {0};	//�������
//static uint8_t Usart1_Tx_Buffer[USART1_TX_LEN] = {0};


/************************** ���� ***************************/
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

static int16_t RC_abs(int16_t value);//ȡ������

/**
  * @brief          ң������
  * @param[in]      none
  * @retval         none
  */
void Remote_Task(void const * argument)
{

		vTaskDelay(5);
		uint8_t remote_ReadBuff[128];

	while(1)
	{
		taskENTER_CRITICAL(); //�����ٽ���

		LEDE5(0);																	//ָʾ��
		
#if (!Debug_mode)		
		if(Get_appinit_status() == CHASSIS_APP)
#endif		
		{
			fifo_read_buff(pfifo_remote,remote_ReadBuff,32);
			sbus_to_rc(remote_ReadBuff,&rc_ctrl);
		}
		 taskEXIT_CRITICAL(); //�˳��ٽ���

		vTaskDelay(2);
	}

	
}


/**
  * @brief      ң����������������
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
  * @brief          ң������ʼ��
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
		hdma_usart1_rx.Instance = DMA1_Stream2;		// ����hdma_usart1_rx.InstanceΪDMA1_Stream2
	
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
	
//		SCB_DisableDCache();
		

}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//�ж�ң���������Ƿ����
uint8_t RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
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
           
//ң�������߻����ݴ���ʱ����ң����
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}



//ȡ������
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
  * @brief          ң����Э�����
  * @param[in]      sbus_buf: ԭ������ָ��
  * @param[out]     rc_ctrl: ң��������ָ
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
		
		rc_ctrl->rc.ch[0] = rc_deadline_limit(rc_ctrl->rc.ch[0], 15); //��������
    rc_ctrl->rc.ch[1] = rc_deadline_limit(rc_ctrl->rc.ch[1], 15); //��������
    rc_ctrl->rc.ch[2] = rc_deadline_limit(rc_ctrl->rc.ch[2], 15); //��������
    rc_ctrl->rc.ch[3] = rc_deadline_limit(rc_ctrl->rc.ch[3], 15); //��������
		
		

}

/**
  * @brief          ң������ʼ��������DMA˫���壩
  * @param[in]      rx1_buf: ������1ָ��
  * @param[in]      rx2_buf: ������2ָ��
  * @param[in]      dma_buf_num: ���������ݴ�С
  * @param[out]     none
  * @retval         none
  */
#if double_buffer
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //ʹ��DMA���ڽ���
    SET_BIT(huart1.Instance ->CR3, USART_CR3_DMAR);

    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //����DMA���䣬������1�����ݰ��˵�recvive_buff��
    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[0], 36 );

    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

	
	/* ((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->CR : H7�ĸĶ���Ҫ�ȶ���hdma_usart1_rx.Instance = DMA1_Stream2; */
    while(DMA1_Stream2->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    DMA1_Stream2->PAR = (uint32_t) & (USART1->RDR);
 
    //�ڴ滺����1
		DMA1_Stream2->M0AR = (uint32_t)(rx1_buf);

    //�ڴ滺����2
		DMA1_Stream2->M1AR = (uint32_t)(rx2_buf);

    //���ݳ���
		DMA1_Stream2->NDTR = dma_buf_num;

    //ʹ��˫������
     SET_BIT(DMA1_Stream2->CR, DMA_SxCR_DBM);

    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

#endif




/**
  * @brief          ң������ʼ����������DMA��
  * @param[in]      none: �����������Ҫ��
  * @param[out]     none
  * @retval         none
  */
#if (!double_buffer)
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //ʹ��DMA���ڽ���
    SET_BIT(huart1.Instance ->CR3, USART_CR3_DMAR);

    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //����DMA���䣬������1�����ݰ��˵�recvive_buff��
//    HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[1], USART1_RX_LEN );
			HAL_UART_Receive_IT(&huart1, Usart1_Rx, USART1_RX_LEN);

	    if (fifo_init(&fifo_usart1_rx, Usart1_Rx_Buffer, USART1_RX_LEN) == -1)
    {
        Error_Handler(); // ���� 2 ���ݴη�
    }
	
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

}
#endif
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart1);
}


/**
  * @brief          ����һ���մ���������DMA��
  * @param[in]      huart: ����һ���
  * @param[out]     none
  * @retval         none
  */
#if (!double_buffer)
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	   volatile uint32_t num = 0;
	    HAL_StatusTypeDef huart_state;			// ��¼���ڽ���״̬
        
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE) == RESET)	//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) != RESET )	// �����ж�
    {
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		}
		
//			HAL_UART_Receive_DMA(&huart1, sbus_rx_buf[1], 32 );
		huart_state =	HAL_UART_Receive_IT(&huart1, Usart1_Rx, 32 );//�ô��ڽ���ң������

		fifo_write_buff(&fifo_usart1_rx, Usart1_Rx, 32);				// д�뻷�ζ���	
}

#endif



/**
  * @brief          ����һ���մ�������DMA˫���壩
  * @param[in]      huart: ����һ���
  * @param[out]     none
  * @retval         ����֪��ΪʲôH7���������ܷ����DMA����һֱ���㣨�ܽ��жϣ�������0����
  */
#if double_buffer
// �����ж�
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	
		LEDE5(0);
	
//	__HAL_UART_CLEAR_IDLEFLAG(huart);		// ��������׼
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE) == RESET)//���յ�����
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

							SCB_InvalidateDCache_by_Addr (sbus_rx_buf[0],SBUS_RX_BUF_NUM);
            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM -  ( __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));//((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
							DMA1_Stream2->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
							DMA1_Stream2->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
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
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            SCB_InvalidateDCache_by_Addr (sbus_rx_buf[1],SBUS_RX_BUF_NUM);

            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM -  ( __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));//((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
             DMA1_Stream2->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
							DMA1_Stream2 ->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);

            }
        }
    }

}	

#endif






//����ң����
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    ((DMA_Stream_TypeDef   *)hdma_usart1_rx.Instance)->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);

}


/**
  * @brief          ҡ��������
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
//	rc_ctrl.rc.s1 = RC_SW_ERROR;  //���ִ���Ϊ0
//	rc_ctrl.rc.s2 = RC_SW_ERROR;  //���ִ���Ϊ0
	rc_ctrl.mouse.x = 0;
	rc_ctrl.mouse.y = 0;
	rc_ctrl.mouse.z = 0;
	rc_ctrl.mouse.press_l = 0;
	rc_ctrl.mouse.press_r = 0;
	rc_ctrl.key.v = 0;
}
