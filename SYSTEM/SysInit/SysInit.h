/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *            ���汣��       ����崻�     ����BUG
 *
 */
#ifndef SYSINIT_H
#define SYSINIT_H

/*  ϵͳͷ�ļ� */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdint.h"

/************************* Task ************************/
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Fire.h"
#include "Task_AboveGimbal.h"

/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
	
/* ************************ Hardward ******************** */	
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_usart2.h"
#include "fifo_buff.h"


/* ************************ CONTROL ******************** */	
#include "CAN_1_Receive.h"
#include "CAN_2_Receive.h"
#include "RemoteControl.h"
#include "FDCAN_Receive.h"
#include "chassis_app.h"
#include "gimbal_app.h"
#include "chassis_behaviour.h"
#include "gimbal_behaviour.h"
#include "automatic_strike.h"
/************************* ALGORITHM ******************** */	

#include "pid.h"
#include "rmmotor.h"
#include "maths.h"
#include "filter.h"

/************************* RefereeDeal ******************** */

#include "RefereeDeal.h"

/************************* SFUD ******************** */
#include "sfud.h"




/* ************************  �ָ���  ******************** */	

#define CHASSIS_APP 0     // ���뿪��ȫ�� ��������Ϊ0��
#define GIMBAL_APP  15    // ���뿪��ȫ�� ��������Ϊ15��            

/***************************** ������ѡ�� *************************************/

#define pitch_angle_position  0   // p��ʹ�ñ�����    		0 -> ��ʹ��  1 -> ʹ��
#define chassis_using		      0   // �����˶�         		0 -> ��ʹ��  1 -> ʹ��
#define hot_limit             0   // ��������         		0 -> ��ʹ��  1 -> ʹ��
#define yaw_angle_limit       1		// yaw ��Ƕ�����				0 -> ������  1 -> ����
#define double_buffer 				0 	// ң��ʹ��˫������			0 -> ��ʹ��	 1 -> ʹ��
#define SFUD_NORFLASH					0		// flash��ʼ����ʹ��		0 -> ��ʹ��  1 -> ʹ��
#define MiniPC_DMA						1		// С������DMAͨ��			0 -> ��ʹ��	 1 ->	ʹ��
#define Debug_mode						0		// ����ģʽ							0 -> ������	 1 -> ����
/*****�������˵���̨��ֵ(����õ�������У׼��Ŀǰ2020��������P���õ���������ʼ��)******/
#define Pitch_Middle_Angle  170 //�ڱ�50   -8  -82     42  -32  74
#define Pitch_UP_Angle      190  //	


//LED�˿ڶ���

#define LEDE2(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,(GPIO_PinState)PIN)	// ����
#define LEDE3(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,(GPIO_PinState)PIN)  // ��̨	
#define LEDE4(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(GPIO_PinState)PIN)   // ����̨
#define LEDE5(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(GPIO_PinState)PIN)	// ң��



//��������Ԫ�صĸ���
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( (a[0]) ) )

/************��� ������*���ٱ� ***************/
#define YAW_RATIO      (5*19)         //Yaw��
#define ABOVE_YAW_RATIO (1*1)
#define PITCH_RATIO		 (1.5*19)       //Pitch��
#define CHASSIS_RATIO  (1*19)					//���̵�����ٱ�
#define Sec_YAW_RATIO  (3*1)          //��Yaw��
#define PITCH_GR			 (1.5)					//pitch������
 
/* ���̵���ƶ��ٶ��趨 */ 
#define M3508_MAX_OUTPUT_CURRENT  10000   //m3508������������  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006������������

#define MAX_MOTOR_CAN_INPUT    2000.0f   //3508����������
#define MAX_MOTOR_CAN_OUTPUT   16000.0f  //3508���������

/*************���ٵ��������������������������**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21


extern void System_init(void);
extern int RNG_Get_RandomRange(int min,int max);
extern uint8_t Get_appinit_status(void);


#endif









