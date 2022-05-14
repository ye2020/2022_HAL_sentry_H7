#ifndef _MYIIC_H
#define _MYIIC_H

#include "main.h"
#include "bsp_dwt.h"
#include "bsp_dwt.h"

//IO��������
#define SDA_IN()  {GPIOD->MODER&=~(3<<(13*2));GPIOH->MODER|=0<<13*2;}	//PD13����ģʽ
#define SDA_OUT() {GPIOD->MODER&=~(3<<(13*2));GPIOH->MODER|=1<<13*2;} //PD13���ģʽ
#define OUT 1
#define IN  0
//IO����
#define IIC_SCL(n)  (n?HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET)) //SCL
#define IIC_SDA(n)  (n?HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET)) //SDA
#define READ_SDA    HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)  //����SDA


extern void IIC_Init(void);
extern void IIC_Start(void);
extern void IIC_Stop(void);
extern uint8_t IIC_Wait_Ack(void);
extern void IIC_NAck(void);
extern void IIC_Ack(void);
extern void IIC_Send_Byte(uint8_t txd);

void IIC_SDA_Mode(uint8_t addr);




#endif

