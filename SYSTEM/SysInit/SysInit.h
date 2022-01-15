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
 *            佛祖保佑       永不宕机     永无BUG
 *
 */
#ifndef SYSINIT_H
#define SYSINIT_H

/*  系统头文件 */
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




/* ************************  分割线  ******************** */	

#define CHASSIS_APP 0     // 拨码开关全上 （即译码为0）
#define GIMBAL_APP  15    // 拨码开关全下 （即译码为15）            

/***************************** 各限制选择 *************************************/

#define pitch_angle_position  0   // p轴使用编码器    		0 -> 不使用  1 -> 使用
#define chassis_using		      0   // 底盘运动         		0 -> 不使用  1 -> 使用
#define hot_limit             0   // 热量限制         		0 -> 不使用  1 -> 使用
#define yaw_angle_limit       1		// yaw 轴角度限制				0 -> 不限制  1 -> 限制
#define double_buffer 				0 	// 遥控使用双缓冲区			0 -> 不使用	 1 -> 使用
#define SFUD_NORFLASH					0		// flash初始化及使能		0 -> 不使能  1 -> 使能
#define MiniPC_DMA						1		// 小电脑用DMA通信			0 -> 不使用	 1 ->	使用
#define Debug_mode						0		// 调试模式							0 -> 不开启	 1 -> 开启
/*****各机器人的云台中值(如果用到编码器校准，目前2020赛季步兵P轴用到编码器初始化)******/
#define Pitch_Middle_Angle  170 //哨兵50   -8  -82     42  -32  74
#define Pitch_UP_Angle      190  //	


//LED端口定义

#define LEDE2(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,(GPIO_PinState)PIN)	// 底盘
#define LEDE3(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,(GPIO_PinState)PIN)  // 云台	
#define LEDE4(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,(GPIO_PinState)PIN)   // 上云台
#define LEDE5(PIN) HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,(GPIO_PinState)PIN)	// 遥控



//返回数组元素的个数
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( (a[0]) ) )

/************电机 传动比*减速比 ***************/
#define YAW_RATIO      (5*19)         //Yaw轴
#define ABOVE_YAW_RATIO (1*1)
#define PITCH_RATIO		 (1.5*19)       //Pitch轴
#define CHASSIS_RATIO  (1*19)					//底盘电机减速比
#define Sec_YAW_RATIO  (3*1)          //副Yaw轴
#define PITCH_GR			 (1.5)					//pitch传动比
 
/* 底盘电机移动速度设定 */ 
#define M3508_MAX_OUTPUT_CURRENT  10000   //m3508电机最大电流输出  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006电机最大电流输出

#define MAX_MOTOR_CAN_INPUT    2000.0f   //3508最大电流输入
#define MAX_MOTOR_CAN_OUTPUT   16000.0f  //3508最大电流输出

/*************减速电机启动电流补偿（快速启动）**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21


extern void System_init(void);
extern int RNG_Get_RandomRange(int min,int max);
extern uint8_t Get_appinit_status(void);


#endif









