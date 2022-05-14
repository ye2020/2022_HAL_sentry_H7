#include "myiic.h"



/**
  * @brief         iic初始化
  * @param[in]      none
  * @retval         none
  */	
void IIC_Init(void)
{
	  IIC_SDA(1);
    IIC_SCL(1);  
}


/**
  * @brief         	SDA方向选择
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
  * @brief         产生起始信号
  * @param[in]      none
  * @retval         none
  */
void IIC_Start(void)
{
		//SDA_OUT();     
		IIC_SDA_Mode(OUT);//sda线输出
		IIC_SDA(1);
    IIC_SCL(1);  
		DWT_delay_us(5);
		IIC_SDA(0);			// 开始信号：SCL保持高电平，SDA由高电平变为低电平后，延时(>4.7us)，SCL变为低电平。
		DWT_delay_us(5);
		IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
}


/**
  * @brief         产生停止信号
  * @param[in]      none
  * @retval         none
  */
void IIC_Stop(void)
{
//		SDA_OUT();
		IIC_SDA_Mode(OUT);//sda线输出
		IIC_SCL(0);
		IIC_SDA(0);//停止信号：SCL保持高电平。SDA由低电平变为高电平。
		DWT_delay_us(5);
		IIC_SCL(1); 
		IIC_SDA(1);//发送I2C总线结束信号
		DWT_delay_us(5);
}


/**
  * @brief         等待应答信号到来
  * @param[in]      none
  * @retval        返回值：1，接收应答失败
	*	        							 0，接收应答成功
  */
uint8_t IIC_Wait_Ack(void)
{
		uint8_t ucErrTime=0;
//		SDA_IN();      
	IIC_SDA_Mode(IN);//SDA设置为输入  
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
		IIC_SCL(0);//时钟输出0 	   
		return 0;  
		
}


/**
  * @brief         产生ACK应答
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
  * @brief         不产生ACK应答
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
  * @brief         IIC发送一个字节
  * @param[in]      none
  * @retval         返回从机有无应答
	*									1，有应答
	*									0，无应答	
  */	  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
//		SDA_OUT(); 	    
		IIC_SDA_Mode(OUT);
    IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7);   // if ((txd << t ) & 0x80)  IIC_SDA(1);
        txd<<=1; 	  							// else IIC_SDA(0);
		DWT_delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL(1);
		DWT_delay_us(2); 
		IIC_SCL(0);	
		DWT_delay_us(2);
    }	 
} 	