/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       automatic_strike.c/h
  * @brief      ���鲿�ִ���
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/

#include "automatic_strike.h"
#include "SysInit.h"


/************************ ���� **************************/

#define USART3_RX_LEN   256
#define USART3_TX_LEN   32

extern UART_HandleTypeDef huart3;                                     // ���������
extern DMA_HandleTypeDef hdma_usart3_rx;                              // DMA���
fifo_rx_def fifo_usart3_rx;                                           // ���������ջ��ζ���
fifo_rx_def fifo_usart3_tx;

static fifo_rx_def *pfifo_visual = &fifo_usart3_rx;                   // С���Ի��ζ���

static uint8_t Usart3_Tx[USART3_TX_LEN] = {0};
static uint8_t Usart3_Rx[USART3_RX_LEN] = {0};												// DMA���ջ�����
static uint8_t Usart3_Rx_Buffer[USART3_RX_LEN] = {0};									// ���ն���
static uint8_t Usart3_Tx_Buffer[USART3_TX_LEN] = {0};

Vision_Auto_Data_t Vision_Auto_Data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};   // ����С���Է��ص�����



/************************ �������� *****************************/
float hex2Float(uint8_t HighByte, uint8_t LowByte);
uint32_t usart3_dma_send(uint8_t *data, uint16_t len);



//�����Ӿ�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
Vision_Auto_Data_t *Get_Auto_Control_Point(void)
{
    return &Vision_Auto_Data;
}



#if MiniPC_DMA
void automatic_init(void)
{
	   
   SET_BIT(huart1.Instance ->CR3, USART_CR3_DMAR); 						//ʹ��DMA���ڽ���

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  							// ʹ�ܿ����ж�

		UART_Start_Receive_DMA(&huart3, Usart3_Rx, USART3_RX_LEN);// ��������DMA���գ��ǵÿ���ѭ��ģʽ��
    if (fifo_init(pfifo_visual, Usart3_Rx_Buffer, USART3_RX_LEN) == -1)
    {
        Error_Handler(); // ���� 2 ���ݴη�
    }

	    /* ʹ�ܴ���DMA���� */
    __HAL_DMA_DISABLE(huart3.hdmatx);  // �رմ���DMA����ͨ�� �����ÿ���ѭ��ģʽ��
    
    if (fifo_init(&fifo_usart3_tx, Usart3_Tx_Buffer, USART3_TX_LEN) == -1)
    {
        Error_Handler(); // ���� 2 ���ݴη�
    }
		
}

#endif
/**
  * @brief          ���������ζ��г�ʼ���������ж� �Ȳ����ѣ�
  * @param[in]      none
  * @retval         none
  * @attention      ���ܣ���ʼ�����ζ��� �������ջ������ͳ��ȵ�
  */
#if (!MiniPC_DMA)
void automatic_init(void)
{

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);  // ʹ�ܿ����ж�

			HAL_UART_Receive_IT(&huart3, Usart3_Rx, USART3_RX_LEN);
    if (fifo_init(pfifo_visual, Usart3_Rx_Buffer, USART3_RX_LEN) == -1)
    {
        Error_Handler(); // ���� 2 ���ݴη�
    }

    /* ʹ�ܴ���DMA���� */
    __HAL_DMA_DISABLE(huart3.hdmatx);  // �رմ���DMA����ͨ�� �����ÿ���ѭ��ģʽ��
    
    if (fifo_init(&fifo_usart3_tx, Usart3_Tx_Buffer, USART3_TX_LEN) == -1)
    {
        Error_Handler(); // ���� 2 ���ݴη�
    }
}
#endif

/**
  * @brief          �������Ӿ��жϺ���(DMA˫����)
  * @param[in]      none
  * @retval         none
  * @attention      ���ܣ��Ӿ����ݽ���,�����ڽ���������
  */
#if MiniPC_DMA
void USER_UART3_IRQHandler(UART_HandleTypeDef *huart)
{
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) == SET)          // ����Ƿ��ǿ����ж�
	{
		  volatile uint32_t num = 0;
			HAL_StatusTypeDef huart_state;																				// ��¼���ڽ���״̬


			__HAL_UART_CLEAR_IDLEFLAG(&huart3);		// ��������׼
			num = huart->Instance->ISR;                                   //���RXNE��־
			num = huart->Instance->RDR;    
		
			__HAL_UART_CLEAR_PEFLAG(&huart3);															 // ���������ɱ�־λ 

			__HAL_DMA_DISABLE(&hdma_usart3_rx);                            // �رմ���DMA����ͨ��
			
			num = USART3_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);  //! ��ȡDMA��δ��������ݸ�����NDTR�Ĵ��������ο����Ĳο��ֲ� ��DMA_Channel_TypeDef��  �����ͬ��оƬHAL�����涨��������е㲻ͬ
			
		
			huart_state = HAL_UART_Receive_DMA(&huart3,Usart3_Rx,USART3_RX_LEN);       // ����DMA
		
				if(huart_state != HAL_OK)																							// �жϽ���״̬����ֹ���
			{
				if(huart_state == HAL_BUSY)
				{
					__HAL_UART_CLEAR_OREFLAG(huart);																 // �������жϱ�־
					huart->RxState = HAL_UART_STATE_READY;													 // ����״̬λ
					huart->Lock    = HAL_UNLOCKED;																	 // ��������
					
					huart_state = HAL_UART_Receive_DMA(&huart3,Usart3_Rx,USART3_RX_LEN); 	//���¿�������
					
				}
			}
			
			fifo_write_buff(pfifo_visual ,Usart3_Rx,num);                // д�뻷�ζ���
			
		__HAL_DMA_ENABLE(&hdma_usart3_rx);														// ����DMA���ǵ�ѽ ������

	}

	
}

#endif

/**
  * @brief          �������Ӿ��жϺ���(�����ж� �Ȳ�����)
  * @param[in]      none
  * @retval         none
  * @attention      ���ܣ��Ӿ����ݽ���,�����ڽ���������
  */
#if (!MiniPC_DMA )
void USER_UART3_IRQHandler(UART_HandleTypeDef *huart)
{

	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) == SET)         						 // ����Ƿ��ǿ����ж�
	{		
    volatile uint32_t num = 0;
		HAL_StatusTypeDef huart_state;																				// ��¼���ڽ���״̬
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);																		// ��������׼
		num = huart->Instance->ISR;           																//���RXNE��־
		num = huart->Instance->RDR;        	 																 //��USART_IT_IDLE��־

		__HAL_UART_CLEAR_PEFLAG(&huart3);		 																 // ���������ɱ�־λ 

                    
		huart_state = HAL_UART_Receive_IT(&huart3, Usart3_Rx, USART3_RX_LEN); // �������ڽ���

		if(huart_state != HAL_OK)																							// �жϽ���״̬����ֹ���
		{
			if(huart_state == HAL_BUSY)
			{
				__HAL_UART_CLEAR_OREFLAG(huart);																 // �������жϱ�־
				huart->RxState = HAL_UART_STATE_READY;													 // ����״̬λ
				huart->Lock    = HAL_UNLOCKED;																	 // ��������
				
				huart_state = HAL_UART_Receive_IT(huart,Usart3_Rx,USART3_RX_LEN);	//���¿�������
				
			}
		}
			fifo_write_buff(pfifo_visual ,Usart3_Rx,16);                // д�뻷�ζ���
  }
}
#endif

/**
  * @brief          ��ȡ���ڽ��յ�����
  * @param[in]      *data: ����ָ��
  * @param[in]      len: ���ݳ���
  * @retval         �ɹ���ȡ�����ݳ���
  */
uint32_t usart3_read(uint8_t *data, uint16_t len)
{
    uint32_t result = 0;

    if (data != NULL)
        result = fifo_read_buff(pfifo_visual, data, len);

    return result;
}


/**
  * @brief      �������ݸ��Ӿ�
  * @param[in]  data: �з�װ�װ�  �죺0 | ����1
  * @param[in]  auto_pitch_angle: p��Ƕ�
  * @param[in]  shoot_speed: ���٣�
  * @retval     none
  * @attention  Э�飺֡ͷ0xFF  ����  ֡β0xFE
  *             ͨ���������ݵ�fifo��������
  *             Ȼ���ڵ��ú����������������ݷŽ�DMA���������棬����DMA����
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
*���ƣ�MiniPC���ڽ������ݴ���
*���ܣ�ͨ�����������ղ�����MiniPC����������
*���룺��
*�������
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
              Vision_Auto_Data.auto_yaw_angle   = (hex2Float(buff_read[i + 2], buff_read[i + 1]) / 100);    //�Զ������y��Ƕȼ���/100
              Vision_Auto_Data.auto_pitch_angle = (hex2Float(buff_read[i + 4], buff_read[i + 3]) / 100);    //�Զ������p��Ƕȼ���
              Vision_Auto_Data.len              = (hex2Float(buff_read[i + 6], buff_read[i + 5]) / 100);    // ����

              i = i + 7;

          }
        }
    }

    else
    {
        // û������
    }

        if (pfifo_visual->error)
    {
        // ���մ���
        pfifo_visual->error = 0;
    }
}


/*
*���ܣ��ߵͰ�λ��������
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
  * @brief          ������ + DMA ����
  * @param[in]      *data
  * @param[in]      *len
  * @retval         void
  */

uint32_t usart3_dma_send(uint8_t *data, uint16_t len)
{	
	uint32_t result = fifo_write_buff(&fifo_usart3_tx, data, len); //�����ݷ���ѭ��������
	
    if (result != 0 && (huart3.gState == HAL_UART_STATE_READY))
    {
        len = fifo_read_buff(&fifo_usart3_tx, Usart3_Tx, USART3_TX_LEN); //��ѭ����������ȡ����
		
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

