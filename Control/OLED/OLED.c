#include "OLED.h"



uint8_t OLED_GRAM[8][128];/*����OLED�Դ�����*/

/**
  * @brief         OLEDд����
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_CMD(uint8_t cmd)
{
		IIC_Start();
		IIC_Send_Byte(OLED);
		IIC_Ack();
		IIC_Send_Byte(0x00);
		IIC_Ack();
		IIC_Send_Byte(cmd);
		IIC_Ack();
		IIC_Stop();
}


/**
  * @brief         OLEDд����
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_Data(uint8_t data)
{
		IIC_Start();
		IIC_Send_Byte(OLED);
		IIC_Ack();
		IIC_Send_Byte(0x40);
		IIC_Ack();
		IIC_Send_Byte(data);
		IIC_Ack();
		IIC_Stop();
}




/**
  * @brief         OLED���
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_Clear(void)
{
		for(uint8_t i = 0 ; i<8; i++)				// д��ÿһҳ������
	{
			OLED_Write_CMD(0xB0 | i);
			OLED_Write_CMD(0x00);
			OLED_Write_CMD(0x10);
		
			for( uint8_t n = 0; n<128 ;n++)		// д��һҳ��128�е�����
		{
				OLED_Write_Data(0x00);					// ����
		}
	}
}


/**
  * @brief         OLED��ʼ��
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_Init(void)
{
		DWT_delay_ms(200);
	
		OLED_Write_CMD(0xA8);			// ���÷ֱ���
		OLED_Write_CMD(0x3F);			// �ֱ��� 128*64
	
		OLED_Write_CMD(0xDA);			// ����COMӲ��������������
		OLED_Write_CMD(0x12);			// [5:4]����
	
		OLED_Write_CMD(0xD3);			// ������ʾƫ��λ��ӳ��RAM������(0x00~0x3F)
		OLED_Write_CMD(0x00);			// Ĭ��00û��ƫ��
	
		OLED_Write_CMD(0x40);			// ������ʼ�е�ַ,��ӳ��RAM��ʾ��ʼ��(0x00~0x3F)
		OLED_Write_CMD(0xA1);			// ���ض�������,bit0:0,0->0;1,0->127; 0xa0���ҷ��� 0xa1����
		OLED_Write_CMD(0x81);			// ���öԱȶȿ��ƼĴ���
		OLED_Write_CMD(0xFF);			// ���ȵ��ڣ�0x00 - 0xFF�� ��ֵԽ������Խ�󣬷����෴
		OLED_Write_CMD(0xA4);			// ȫ����ʾ����;bit0:1,����;0,�ر�;(����/����) (0xa4/0xa5)
		OLED_Write_CMD(0xA6);			// ������ʾ��ʽ;bit0:1,������ʾ;0,������ʾ (0xa6/0xa7) 
		OLED_Write_CMD(0xD5);			// ������ʾʱ�ӷ�Ƶ��/����Ƶ��
		OLED_Write_CMD(0xF0);			// ���÷ֱ���ֵ
		OLED_Write_CMD(0x8D);			// ���ó�������/����
		OLED_Write_CMD(0x14);			// ����(0x10����,0x14����)
		OLED_Write_CMD(0xAE);			// ��ʾ�ر� 0xAF�������� 0xAE �� �ر�
		OLED_Write_CMD(0x20);			// ����ҳ��Ѱַģʽ(0x00/0x01/0x02)(ˮƽ/��ֱ/ҳѰַ)
		OLED_Write_CMD(0x02);			// ҳѰַ0x02
		OLED_Write_CMD(0xB0);			// ΪҳѰַģʽ����ҳ�濪����ַ0-7
		OLED_Write_CMD(0xC8);			// ����comɨ�跽ʽ 0xc0���·��� 0xc8����
		OLED_Write_CMD(0x00);			// ���õ��е�ַ
		OLED_Write_CMD(0x10);			// ���ø��е�ַ
		OLED_Write_CMD(0x40);			// ������ʼ�е�ַ,��ӳ��RAM��ʾ��ʼ��(0x00~0x3F)
		OLED_Write_CMD(0xD9);			// ����Ԥ�������
		OLED_Write_CMD(0x22);			// ���ʱ��
		OLED_Write_CMD(0xDB);			// ����VCOMH ��ѹ����
		OLED_Write_CMD(0x20);			// Set VCOM �ͷŵ�ѹ([6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;) Ĭ��0x20 0.77*vcc
		OLED_Write_CMD(0xAF);			// ��ʾ����
		
		OLED_Write_Off();
		OLED_Write_Clear();

}


/**
  * @brief         OLED����
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_On(void)
{
		OLED_Write_CMD(0x8D);		// ���ó�������/����
		OLED_Write_CMD(0x14);		// ������ɱ�
		OLED_Write_CMD(0xAF);		// ��ʾ����
}


/**
  * @brief         OLED����
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_Off(void)
{
		OLED_Write_CMD(0x8D);		// ���ó�������/����
		OLED_Write_CMD(0x10);		// ������ɱ�
		OLED_Write_CMD(0xAE);		// ��ʾ����
}



/*
�������ܣ����Դ������ϻ�һ����
����������x��yΪ��ĺ�������   cΪ����������1��0��
������Χ��x 0~128  y 0~8 
ÿһ�������� ��λ��ǰ����λ�ں�
*/
void OLED_Draw_Point(uint8_t x,uint8_t y,uint8_t c)
{
		uint8_t page,addr;
		page = y/8; //�Դ��ҳ��ַ
		addr = y%8; //�Դ��һ���ֽ�������c���ڵ�λ�� 
		if(c) OLED_GRAM[page][x] |= 1<<addr;
		else  OLED_GRAM[page][x] &= ~(1<<addr);
}





/**
  * @brief         	���ù��λ��
  * @param[in]      x�е���ʼλ��(0~127)		
	*								  yҳ����ʼλ��(0~7)    ����: 0x8  ��4λ0000   ��4λ1000 
  * @retval         none
  */
void OLED_SetCursorAddrese(uint8_t x,uint8_t y)
{
		OLED_Write_CMD(0xB0+y); 					//����ҳ��ַ
		OLED_Write_CMD((x&0xF0)>>4|0x10);//�����и���ʼ��ַ(���ֽ�)
		OLED_Write_CMD((x&0x0F)|0x00);   //�����е���ʼ��ַ(���ֽ�)			
}

/**
  * @brief         OLED��ʾ�ַ�
	* @param[in]      	x: ��ʾҳ  y����ʾ��		width���ֿ�   height���ָ�    data������
  * @retval         
  */
uint8_t Oled_Display(uint8_t x ,uint8_t y ,uint8_t width , uint8_t height ,const uint8_t *data)
{
		uint8_t i,j;
		if(width>128) width=128;
		if(height>64) height=64;
	
		/*1. OLED��ʾ����ʼ��*/
		OLED_Write_Clear();	//����
		
		for(i=0;	i<height	;i++)
		{
			OLED_Write_CMD((0xB0 | i) + x); 	 //����ҳ��ַ��0~7��
			OLED_Write_CMD(0x10 + (y >> 4 & 0x0f));      //������ʾλ�èD�е�ַ
			OLED_Write_CMD(y & 0x0f);       
//				OLED_Write_CMD(0xB0 + i);
//				OLED_Write_CMD(0x00);
//				OLED_Write_CMD(0x10);
			
			for(j=0;j<width;j++)
			{ 
				OLED_Write_Data(*data++);
			}
		}
		DWT_delay_ms(10);
		return 0;
}



/**
  * @brief         OLED��ʾ�ַ���
* @param[in]      	x: ��ʾҳ  y����ʾ��		width���ֿ�   height���ָ�    str������
  * @retval         none
  */
void OLED_DisplayString(uint8_t x,uint8_t y,uint8_t width,uint8_t height,const uint8_t *str)
{
		uint8_t  addr=0,i;				// addr -> �ֿ���   
		uint16_t font=0;
	
		while(*str!='\0') //������ʾ
		{
				//ȡģ�ӿո�ʼ�ģ������±�

	/************************************************** ��ʾӢ�� ********************************************/
			if(*str >= ' '&& *str <= '~') 	
			{
					addr = *str - ' '; //ȡģ�ӿո�ʼ�ģ������±�
				
				
					/*********************** д8*16ASCII�ַ����ϰ벿�� ***************************/
				
				{
						OLED_SetCursorAddrese(x,y);			//���ù���λ��
					
						for(i=0;i<width/2;i++)      //����дwidth��
					{
						OLED_Write_Data(ASCII_8_16[addr][i]);
					}			
				}
				
					/*********************** д8*16ASCII�ַ����°벿�� ***************************/
				
				{
						OLED_SetCursorAddrese(x,y+1); //���ù���λ��
					
							for(i=0;i<width/2;i++)        //����дwidth��
						{
							 OLED_Write_Data(ASCII_8_16[addr][i+width/2]); 
						}	
				}
				
						str++;//������ʾ��һ���ַ�
						x+=width/2; //����һ��λ�ü�����ʾ����
			}
				
		/************************************************** ��ʾ���� ********************************************/
	
			else
			{
				OLED_SetCursorAddrese(x,y); 	//���ù���λ��
				font=((*str)<<8)+(*(str+1));	// ��������
				
				for(uint8_t j = 0 ; j < (sizeof(CN_16_16)/ sizeof(CN_16_16[0])) ;j++)		// ������
				{
					if(font == CN_16_16[j].Index)
					{
					/*********************** д8*16ASCII�ַ����ϰ벿�� ***************************/

						for(i=0;i<width;i++) //����дwidth��
						{
							OLED_Write_Data(CN_16_16[j].Msk[i]);
						}
						
					/*********************** д8*16ASCII�ַ����°벿�� ***************************/
						
						OLED_SetCursorAddrese(x,y+1); //���ù���λ��
						
						for(i=0;i<width;i++)
						{
							OLED_Write_Data(CN_16_16[j].Msk[i + width]);
						}				
					}
				}
					str+=2;//������ʾ��һ���ַ�
					x+=width; //����һ��λ�ü�����ʾ����
			}
				
		}
	
}


/*****************************************************
** �������ƣ�OLED_VerticalDisplay
** �������ܣ�ʵ��OLEDˮƽ������Χ����
** ��    ����1.toprow�����ù�����ʼ��
**           2.scrollrow�����ù�������
**           ע�⣺toprow+scrollrow<64
** �������أ���
******************************************************/
void OLED_HorizontalDisplay(void)
{
	OLED_Write_CMD(0x2E);					// �رչ���
	OLED_Write_CMD(0x27);					// ˮƽ��������ҹ� ��26/27��
	OLED_Write_CMD(0x00);					// �����ֽ�
	OLED_Write_CMD(0x00);					// ��ʼҳ 0
	OLED_Write_CMD(0x07);					// ����ʱ����
	OLED_Write_CMD(0x01);					// ��ֹҳ 7
	OLED_Write_CMD(0x00);					// �����ֽ�
	OLED_Write_CMD(0xFF);					// �����ֽ�
}

/*****************************************************
** �������ƣ�OLED_VerticalDisplay
** �������ܣ�ʵ��OLEDˮƽ������Χ����
** ��    ����1.toprow�����ù�����ʼ��
**           2.scrollrow�����ù�������
**           ע�⣺toprow+scrollrow<64
** �������أ���
******************************************************/
void OLED_VerticalDisplay(void)
{
	OLED_Write_CMD(0x2E);					// �رչ���
	OLED_Write_CMD(0x29);					// ��ֱ���� ��29/2a��
	OLED_Write_CMD(0x00);					// �����ֽ�
	OLED_Write_CMD(0x00);					// ��ʼҳ 0
	OLED_Write_CMD(0x07);					// ����ʱ����
	OLED_Write_CMD(0x01);					// ��ֹҳ 7 (ˮƽ)
	OLED_Write_CMD(0x01);					// ����ƫ��������ֱ��
}


/*****************************************************
** �������ƣ�OledScrollStop
** �������ܣ�������������
******************************************************/ 
void OledScrollStart(void)
{
	OLED_Write_CMD(0x2F);
}