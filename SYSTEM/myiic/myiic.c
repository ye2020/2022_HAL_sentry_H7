#include "myiic.h"



/**
  * @brief         iic��ʼ��
  * @param[in]      none
  * @retval         none
  */	
void IIC_Init(void)
{
	  IIC_SDA(1);
    IIC_SCL(1);  
}


/**
  * @brief         	SDA����ѡ��
  * @param[in]      none
  * @retval         none
  */	
void IIC_SDA_Mode(uint8_t addr)
{
		GPIO_InitTypeDef GPIO_InitStruct;
	
	if(addr)   // OUT -> 1
	{
		GPIO_InitStruct.Pin = GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}
	
	else			// IN -> 0
	{
		GPIO_InitStruct.Pin = GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}

}

/**
  * @brief         ������ʼ�ź�
  * @param[in]      none
  * @retval         none
  */
void IIC_Start(void)
{
		//SDA_OUT();     
		IIC_SDA_Mode(OUT);//sda�����
		IIC_SDA(1);
    IIC_SCL(1);  
		DWT_delay_us(5);
		IIC_SDA(0);			// ��ʼ�źţ�SCL���ָߵ�ƽ��SDA�ɸߵ�ƽ��Ϊ�͵�ƽ����ʱ(>4.7us)��SCL��Ϊ�͵�ƽ��
		DWT_delay_us(5);
		IIC_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}


/**
  * @brief         ����ֹͣ�ź�
  * @param[in]      none
  * @retval         none
  */
void IIC_Stop(void)
{
//		SDA_OUT();
		IIC_SDA_Mode(OUT);//sda�����
		IIC_SCL(0);
		IIC_SDA(0);//ֹͣ�źţ�SCL���ָߵ�ƽ��SDA�ɵ͵�ƽ��Ϊ�ߵ�ƽ��
		DWT_delay_us(5);
		IIC_SCL(1); 
		IIC_SDA(1);//����I2C���߽����ź�
		DWT_delay_us(5);
}


/**
  * @brief         �ȴ�Ӧ���źŵ���
  * @param[in]      none
  * @retval        ����ֵ��1������Ӧ��ʧ��
	*	        							 0������Ӧ��ɹ�
  */
uint8_t IIC_Wait_Ack(void)
{
		uint8_t ucErrTime=0;
//		SDA_IN();      
	IIC_SDA_Mode(IN);//SDA����Ϊ����  
		IIC_SDA(1);DWT_delay_us(1);	   
		IIC_SCL(1);DWT_delay_us(1);	
		
		while(READ_SDA)
		{
			ucErrTime++;
			if(ucErrTime>250)
			{
				IIC_Stop();
				return 1;
			}
		}
		IIC_SCL(0);//ʱ�����0 	   
		return 0;  
		
}


/**
  * @brief         ����ACKӦ��
  * @param[in]      none
  * @retval         none
  */
void IIC_Ack(void)
{
		IIC_SCL(0);
//		SDA_OUT();
	IIC_SDA_Mode(OUT);
		IIC_SDA(0);
		DWT_delay_us(2);
		IIC_SCL(1);
		DWT_delay_us(2);
		IIC_SCL(0);
}


/**
  * @brief         ������ACKӦ��
  * @param[in]      none
  * @retval         none
  */
void IIC_NAck(void)
{
		IIC_SCL(0);
//		SDA_OUT();
		IIC_SDA_Mode(OUT);
		IIC_SDA(1);
		DWT_delay_us(2);
		IIC_SCL(1);
		DWT_delay_us(2);
		IIC_SCL(0);
}		


/**
  * @brief         IIC����һ���ֽ�
  * @param[in]      none
  * @retval         ���شӻ�����Ӧ��
	*									1����Ӧ��
	*									0����Ӧ��	
  */	  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
//		SDA_OUT(); 	    
		IIC_SDA_Mode(OUT);
    IIC_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7);   // if ((txd << t ) & 0x80)  IIC_SDA(1);
        txd<<=1; 	  							// else IIC_SDA(0);
		DWT_delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL(1);
		DWT_delay_us(2); 
		IIC_SCL(0);	
		DWT_delay_us(2);
    }	 
} 	