/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			remote_control.h
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

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "main.h"
#include "usart.h"
#include "stdio.h"


#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
    } key;

} RC_ctrl_t;

void remote_control_init(void);
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
extern const RC_ctrl_t *get_remote_control_point(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void sbus_to_usart1(uint8_t *sbus);
void Remote_reload(void);
uint8_t RC_data_is_error(void);



#endif
