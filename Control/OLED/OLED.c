#include "OLED.h"



uint8_t OLED_GRAM[8][128];/*定义OLED显存数组*/

/**
  * @brief         OLED写命令
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
  * @brief         OLED写数据
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
  * @brief         OLED清除
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_Clear(void)
{
		for(uint8_t i = 0 ; i<8; i++)				// 写入每一页的内容
	{
			OLED_Write_CMD(0xB0 | i);
			OLED_Write_CMD(0x00);
			OLED_Write_CMD(0x10);
		
			for( uint8_t n = 0; n<128 ;n++)		// 写入一页中128列的内容
		{
				OLED_Write_Data(0x00);					// 清零
		}
	}
}


/**
  * @brief         OLED初始化
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_Init(void)
{
		DWT_delay_ms(200);
	
		OLED_Write_CMD(0xA8);			// 设置分辨率
		OLED_Write_CMD(0x3F);			// 分辨率 128*64
	
		OLED_Write_CMD(0xDA);			// 设置COM硬件引脚配置命令
		OLED_Write_CMD(0x12);			// [5:4]配置
	
		OLED_Write_CMD(0xD3);			// 设置显示偏移位移映射RAM计数器(0x00~0x3F)
		OLED_Write_CMD(0x00);			// 默认00没有偏移
	
		OLED_Write_CMD(0x40);			// 设置起始行地址,集映射RAM显示起始行(0x00~0x3F)
		OLED_Write_CMD(0xA1);			// 段重定义设置,bit0:0,0->0;1,0->127; 0xa0左右反置 0xa1正常
		OLED_Write_CMD(0x81);			// 设置对比度控制寄存器
		OLED_Write_CMD(0xFF);			// 亮度调节（0x00 - 0xFF） 数值越大亮度越大，否则相反
		OLED_Write_CMD(0xA4);			// 全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏) (0xa4/0xa5)
		OLED_Write_CMD(0xA6);			// 设置显示方式;bit0:1,反相显示;0,正常显示 (0xa6/0xa7) 
		OLED_Write_CMD(0xD5);			// 设置显示时钟分频比/振荡器频率
		OLED_Write_CMD(0xF0);			// 设置分辨率值
		OLED_Write_CMD(0x8D);			// 设置充电泵启用/禁用
		OLED_Write_CMD(0x14);			// 设置(0x10禁用,0x14启用)
		OLED_Write_CMD(0xAE);			// 显示关闭 0xAF：开启； 0xAE ： 关闭
		OLED_Write_CMD(0x20);			// 设置页面寻址模式(0x00/0x01/0x02)(水平/垂直/页寻址)
		OLED_Write_CMD(0x02);			// 页寻址0x02
		OLED_Write_CMD(0xB0);			// 为页寻址模式设置页面开启地址0-7
		OLED_Write_CMD(0xC8);			// 设置com扫描方式 0xc0上下反置 0xc8正常
		OLED_Write_CMD(0x00);			// 设置低列地址
		OLED_Write_CMD(0x10);			// 设置高列地址
		OLED_Write_CMD(0x40);			// 设置起始行地址,集映射RAM显示起始行(0x00~0x3F)
		OLED_Write_CMD(0xD9);			// 设置预充电周期
		OLED_Write_CMD(0x22);			// 充电时间
		OLED_Write_CMD(0xDB);			// 设置VCOMH 电压倍率
		OLED_Write_CMD(0x20);			// Set VCOM 释放电压([6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;) 默认0x20 0.77*vcc
		OLED_Write_CMD(0xAF);			// 显示开启
		
		OLED_Write_Off();
		OLED_Write_Clear();

}


/**
  * @brief         OLED亮屏
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_On(void)
{
		OLED_Write_CMD(0x8D);		// 设置充电泵启用/禁用
		OLED_Write_CMD(0x14);		// 开启电荷泵
		OLED_Write_CMD(0xAF);		// 显示开启
}


/**
  * @brief         OLED亮屏
  * @param[in]      none
  * @retval         none
  */
void OLED_Write_Off(void)
{
		OLED_Write_CMD(0x8D);		// 设置充电泵启用/禁用
		OLED_Write_CMD(0x10);		// 开启电荷泵
		OLED_Write_CMD(0xAE);		// 显示开启
}



/*
函数功能：在显存数组上画一个点
函数参数：x，y为点的横纵坐标   c为这个点的亮灭（1亮0灭）
参数范围：x 0~128  y 0~8 
每一个数据是 低位在前，高位在后
*/
void OLED_Draw_Point(uint8_t x,uint8_t y,uint8_t c)
{
		uint8_t page,addr;
		page = y/8; //显存的页地址
		addr = y%8; //显存的一个字节数据中c所在的位置 
		if(c) OLED_GRAM[page][x] |= 1<<addr;
		else  OLED_GRAM[page][x] &= ~(1<<addr);
}





/**
  * @brief         	设置光标位置
  * @param[in]      x列的起始位置(0~127)		
	*								  y页的起始位置(0~7)    比如: 0x8  高4位0000   低4位1000 
  * @retval         none
  */
void OLED_SetCursorAddrese(uint8_t x,uint8_t y)
{
		OLED_Write_CMD(0xB0+y); 					//设置页地址
		OLED_Write_CMD((x&0xF0)>>4|0x10);//设置列高起始地址(半字节)
		OLED_Write_CMD((x&0x0F)|0x00);   //设置列低起始地址(半字节)			
}

/**
  * @brief         OLED显示字符
	* @param[in]      	x: 显示页  y：显示列		width：字宽   height：字高    data：数据
  * @retval         
  */
uint8_t Oled_Display(uint8_t x ,uint8_t y ,uint8_t width , uint8_t height ,const uint8_t *data)
{
		uint8_t i,j;
		if(width>128) width=128;
		if(height>64) height=64;
	
		/*1. OLED显示屏初始化*/
		OLED_Write_Clear();	//清屏
		
		for(i=0;	i<height	;i++)
		{
			OLED_Write_CMD((0xB0 | i) + x); 	 //设置页地址（0~7）
			OLED_Write_CMD(0x10 + (y >> 4 & 0x0f));      //设置显示位置―列地址
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
  * @brief         OLED显示字符串
* @param[in]      	x: 显示页  y：显示列		width：字宽   height：字高    str：数据
  * @retval         none
  */
void OLED_DisplayString(uint8_t x,uint8_t y,uint8_t width,uint8_t height,const uint8_t *str)
{
		uint8_t  addr=0,i;				// addr -> 字库编号   
		uint16_t font=0;
	
		while(*str!='\0') //连续显示
		{
				//取模从空格开始的，计算下标

	/************************************************** 显示英文 ********************************************/
			if(*str >= ' '&& *str <= '~') 	
			{
					addr = *str - ' '; //取模从空格开始的，计算下标
				
				
					/*********************** 写8*16ASCII字符的上半部分 ***************************/
				
				{
						OLED_SetCursorAddrese(x,y);			//设置光标的位置
					
						for(i=0;i<width/2;i++)      //横向写width列
					{
						OLED_Write_Data(ASCII_8_16[addr][i]);
					}			
				}
				
					/*********************** 写8*16ASCII字符的下半部分 ***************************/
				
				{
						OLED_SetCursorAddrese(x,y+1); //设置光标的位置
					
							for(i=0;i<width/2;i++)        //横向写width列
						{
							 OLED_Write_Data(ASCII_8_16[addr][i+width/2]); 
						}	
				}
				
						str++;//继续显示下一个字符
						x+=width/2; //在下一个位置继续显示数据
			}
				
		/************************************************** 显示中文 ********************************************/
	
			else
			{
				OLED_SetCursorAddrese(x,y); 	//设置光标的位置
				font=((*str)<<8)+(*(str+1));	// 求汉字内码
				
				for(uint8_t j = 0 ; j < (sizeof(CN_16_16)/ sizeof(CN_16_16[0])) ;j++)		// 查数组
				{
					if(font == CN_16_16[j].Index)
					{
					/*********************** 写8*16ASCII字符的上半部分 ***************************/

						for(i=0;i<width;i++) //横向写width列
						{
							OLED_Write_Data(CN_16_16[j].Msk[i]);
						}
						
					/*********************** 写8*16ASCII字符的下半部分 ***************************/
						
						OLED_SetCursorAddrese(x,y+1); //设置光标的位置
						
						for(i=0;i<width;i++)
						{
							OLED_Write_Data(CN_16_16[j].Msk[i + width]);
						}				
					}
				}
					str+=2;//继续显示下一个字符
					x+=width; //在下一个位置继续显示数据
			}
				
		}
	
}


/*****************************************************
** 函数名称：OLED_VerticalDisplay
** 函数功能：实现OLED水平滚动范围配置
** 参    数：1.toprow：设置滚动起始行
**           2.scrollrow：设置滚动行数
**           注意：toprow+scrollrow<64
** 函数返回：无
******************************************************/
void OLED_HorizontalDisplay(void)
{
	OLED_Write_CMD(0x2E);					// 关闭滚动
	OLED_Write_CMD(0x27);					// 水平向左或向右滚 （26/27）
	OLED_Write_CMD(0x00);					// 虚拟字节
	OLED_Write_CMD(0x00);					// 起始页 0
	OLED_Write_CMD(0x07);					// 滚动时间间隔
	OLED_Write_CMD(0x01);					// 终止页 7
	OLED_Write_CMD(0x00);					// 虚拟字节
	OLED_Write_CMD(0xFF);					// 虚拟字节
}

/*****************************************************
** 函数名称：OLED_VerticalDisplay
** 函数功能：实现OLED水平滚动范围配置
** 参    数：1.toprow：设置滚动起始行
**           2.scrollrow：设置滚动行数
**           注意：toprow+scrollrow<64
** 函数返回：无
******************************************************/
void OLED_VerticalDisplay(void)
{
	OLED_Write_CMD(0x2E);					// 关闭滚动
	OLED_Write_CMD(0x29);					// 垂直滚动 （29/2a）
	OLED_Write_CMD(0x00);					// 虚拟字节
	OLED_Write_CMD(0x00);					// 起始页 0
	OLED_Write_CMD(0x07);					// 滚动时间间隔
	OLED_Write_CMD(0x01);					// 终止页 7 (水平)
	OLED_Write_CMD(0x01);					// 滚动偏移量（垂直）
}


/*****************************************************
** 函数名称：OledScrollStop
** 函数功能：开启滚屏功能
******************************************************/ 
void OledScrollStart(void)
{
	OLED_Write_CMD(0x2F);
}