/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       chassis_task.c/h
  * @brief      完成底盘控制任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
#ifndef TASK_CHASSIS_H
#define TASK_CHASSIS_H


#include "RemoteControl.h"
#include "CAN_1_Receive.h"
#include "pid.h"
#include "maths.h"

/******************************  宏定义 ****************************************/
/*OS控制任务周期以及启动时间*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME   2

/**********************低通滤波比例**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K  0.0510f  //0.0110f 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高  |  0.26f  |  0.0097f

/**********************底盘电机pid参数**********************/

//底盘电机速度参数
#define CHASSIS_MOTOR_PID_Kp    8.0f
#define CHASSIS_MOTOR_PID_Ki    2.0f
#define CHASSIS_MOTOR_PID_Kd    0.0f

//位置环参数
#define CHASSIS_LOCATION_PID_P  100.0f
#define CHASSIS_LOCATION_PID_I  10.0f
#define CHASSIS_LOCATION_PID_D  10.8f

// 方向
#define	GOFORWARD	 1				//自动模式下的向前
#define	GOBACK		 0				//自动模式下的向后

// 激光传感器
#define HAVE_THING 1		//距离内有东西0     
#define NO_THING   0		//距离内没东西1


#define SPEED_TIME_FLAG				430	//510			//变速时间
#define SPEED_MODE					4//底盘变速：1 -> 不变速； 2 -> 变速  ; 3 -> 变速 + 变向1（随机变向） ;4 -> 变向2（等差数列）
/**********************运动加速度限制**********************/
#define STRAIGHT_ACCELERAD        3.5f      //直行底盘加速度限制
#define TRANSLATION_ACCELERAD     5.5f      //平移底盘加速度限制


#define CHASSIS_AUTO_SPPED				4000			 //21赛季为7000			
#define CHASSIS_BLOCKING_SPPED		    6700 //走位速度 ，挨打后的速度 7000  
#define CHASSIS_AUTO_SLOW_SPPED			0    // 遇到敌人 速度减慢		5.17   3000->0  5.25

/***********************************  结构体 *************************************/

/*底盘电机数据*/
typedef struct
{
    const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    fp32 position;
    fp32 last_position;
    int16_t output;
} Motor_t;


/*底盘模式*/
typedef enum
{
    CHASSIS_STOP,       //停止
    CHASSIS_INITIALIZE, //初始化中
    CHASSIS_STANDBY,    //待机

    //CHASSIS_WORKING,   //工作

    CHASSIS_FOLLOW,    //跟随
    CHASSIS_NO_FOLLOW, //不跟随

    CHASSIS_TWIST_WAIST, //扭腰
    CHASSIS_ROTATION,    //小陀螺
    CHASSIS_BATTERY,     //炮台模式

    CHASSIS_REMOTECONTROL,//遥控模式
    CHASSIS_AUTO,				   //自动模式
    CHASSIS_BLOCKING,      //走位模式
    CHASSIS_FIXATION			// 底盘固定模式
} Chassis_mode_e;


/*敌人颜色*/
typedef enum
{
    Enemy_color_red = 0,    //敌人颜色为红色
    Enemy_color_blue		//敌人颜色为蓝色

} Enemy_color_e;


/*底盘整体数据结构体*/
typedef struct
{
   const  RC_ctrl_t *chassis_RC;             //底盘使用的遥控器指针
	
//    const gimbal_yaw_receive_t *gimbal_re_data; //云台板处理数据，数据由can2传输给底盘板，底盘板再输出给yaw轴电机
//    const Fire_task_t *Fire_task_control;
//	
    motor_measure_t *yaw_motor_measure;      //can1直接接收的yaw轴数据 （用于发给云台处理）
    motor_measure_t *chassis_motor_measure;

//    PowerLimit_t Chassis_PowerLimit; //底盘功率限制结构体
    Motor_t chassis_motor[4];        //底盘电机数据(包含电机统一结构体指针)
    PidTypeDef chassis_speed_pid[4]; //正常移动pid
    PidTypeDef chassis_location_pid; //底盘位置环pid

    Chassis_mode_e chassis_mode; //底盘控制状态机

    first_order_filter_type_t LowFilt_chassis_vx; //低通滤波器
    first_order_filter_type_t LowFilt_chassis_vy; //低通滤波器

    fp32 speed_x_set; //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 speed_y_set; //底盘设定速度 左右方向 左为正，单位 m/s

    fp32 chassis_gimbal_angel; //底盘与云台的角度

    uint8_t sign;								//前后走标志
    uint8_t sign_last;						//延续前后走标志

} chassis_control_t;



/********************************* 函数声明 ***********************************/

extern  void Chassis_Task(void const * argument);
extern  void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2);
extern  void Chassis_Task_OFF(uint8_t options);
extern uint8_t  automatic_Enemy_color(void);


#endif





