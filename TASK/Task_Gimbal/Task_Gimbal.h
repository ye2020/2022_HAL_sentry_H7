/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       gimbal_task.c/h
  * @brief      云台任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
	
#ifndef TASK_GIMBAL_H
#define TASK_GIMBAL_H



#include "RemoteControl.h"
#include "CAN_1_Receive.h"
#include "pid.h"
#include "maths.h"
#include "automatic_strike.h"
#include "Task_Fire.h"

/******************************  宏定义 ****************************************/


#define PITCH_PID_MODE            2 // 模式1： 分段pid ，pitch轴上下pid分开 ： 模式2 合并pid


/*********************** 遥控速度设置 *******************************/
#define RC_YAW_SPEED          0.0026f   //遥控器yaw轴速度增益
#define RC_PITCH_SPEED        0.0026f   //遥控器pitch轴速度增益 0.0026

/**********************云台pitch和yaw角度限制**********************/
#define PITCH_ANGLE_LIMIT_UP    30
#define PITCH_ANGLE_LIMIT_DOWN  (-15)
#define YAW_ANGLE_LIMIT         15

/**********************pitch和yaw输出量限制**********************/
#define YAW_OUTPUT_LIMIT         11000
#define YAW_INIT_OUTPUT_LIMIT    8000
#define PITCH_OUTPUT_LIMIT       8000
#define PITCH_INIT_OUTPUT_LIMIT  5000

/**********************低通滤波比例**********************/
#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0884f


/*OS控制任务周期以及启动时间*/
#define GIMBAL_TASK_INIT_TIME 	5
#define GIMBAL_CONTROL_TIME_MS  2

/********************** P轴pid参数 **********************/

#define GIMBAL_P_YAW_P  130.0f   //Y位置环    150     62    130
#define GIMBAL_P_YAW_I  0.0f
#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_S_YAW_P  9.0f     //Y速度环     8      10     9
#define GIMBAL_S_YAW_I  0.0f
#define GIMBAL_S_YAW_D  0.0f     //2                   0


#if (PITCH_PID_MODE == 1)

    #define GIMBAL_UP_P_PITCH_P  40.5f   //P位置环  53   410    160       58  41.5
    #define GIMBAL_UP_P_PITCH_I  0.0f     //8f
    #define GIMBAL_UP_P_PITCH_D  3.8f     //0.0f       360       100     100

    #define GIMBAL_UP_S_PITCH_P  20.5f   //P速度环(不要加i)       1.5     9.5
    #define GIMBAL_UP_S_PITCH_I  0.0f
    #define GIMBAL_UP_S_PITCH_D  3.8f

    #define GIMBAL_DOWN_P_PITCH_P  38.0f   //P位置环  53   410    160       58
    #define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f
    #define GIMBAL_DOWN_P_PITCH_D  3.8f     //0.0f       360       100     100

    #define GIMBAL_DOWN_S_PITCH_P  10.5f   //P速度环(不要加i)       1.5     9.5
    #define GIMBAL_DOWN_S_PITCH_I  0.0f
    #define GIMBAL_DOWN_S_PITCH_D  3.8f

#elif (PITCH_PID_MODE == 2)


    #define GIMBAL_P_PITCH_P  60.0f         //P位置环  70
    #define GIMBAL_P_PITCH_I 	1.5f		// 1.1f
    #define GIMBAL_P_PITCH_D  75.0f         //0.0f   75

    #define GIMBAL_S_PITCH_P  10.5f     //P速度环(不要加i) 1.5     9.5
    #define GIMBAL_S_PITCH_I  0.0f
    #define GIMBAL_S_PITCH_D  6.0f	    //6.0f    //3.5  4

#endif

/*********视觉PY轴数据pid参数定义***************/

#if (PITCH_PID_MODE == 1)
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_P 57.0f    //P自动位置环  530.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_D 3.0f    //200.0f

    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_P 25.0f     //P自动速度环
    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_D 3.0f

    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_P 20.0f    //P自动位置环  530.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_D 0.0f    //200.0f

    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_P 10.0f     //P自动速度环
    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_D 0.0f


#elif (PITCH_PID_MODE == 2)
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_P 36.0f    //P自动位置环  30.0f |530.0f |35 |  30
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_I 0.1f
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_D 9.0f     //200.0f    9.0

    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_P 10.0f     //P自动速度环  12  8
    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      //5
#endif

    #define GIMBAL_AUTO_INDUSTRY_P_YAW_P 100.0f      //Y自动位置环 63.0f |60.0f |170
    #define GIMBAL_AUTO_INDUSTRY_P_YAW_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_P_YAW_D 15.0f

    #define GIMBAL_AUTO_INDUSTRY_S_YAW_P 15.0f       //Y自动速度环 12
    #define GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_S_YAW_D 1.0f



/***********************************  结构体 *************************************/



typedef enum
{
    GIMBAL_STOP,         //停止
    GIMBAL_INITIALIZE,   //初始化状态
    GIMBAL_STANDBY,      //待机状态

    GIMBAL_WORKING,      //初始化结束位,暂时没什么用
    GIMBAL_REMOTECONTROL,       //手动状态

    GIMBAL_REMOTECONTROL_STOP_SHOOT,       // 遥控停止射击状态
    GIMBAL_REMOTECONTROL_LOW_SPEED,        // 遥控低射速状态
    GIMBAL_REMOTECONTROL_HIGH_SPEED,       // 遥控高射速状态

    GIMBAL_AUTOCONTROL,  //比赛模式
    GIMBAL_PATROl,       //巡逻状态
    GIMBAL_AUTOATTACK,   //自瞄状态

    GIMBAL_DOUBLE_GIMBAL,//双云台状态（暂无，之后可能有）

} gimbal_behaviour_e;


typedef enum
{
	Enemy_Appear=0,		        //发现敌人
	Enemy_Disappear=1,			//敌人消失
}VisionStatus_E;		        //在自动控制模式里面使用



typedef struct  //申明pitch轴电机变量
{
    motor_measure_t *pitch_motor_measure;

    #if (PITCH_PID_MODE == 1)

    PidTypeDef pitch_up_p_pid;  //pid
    PidTypeDef pitch_up_s_pid;  //pid

    PidTypeDef pitch_down_p_pid;  //pid
    PidTypeDef pitch_down_s_pid;  //pid

    PidTypeDef pitch_down_auto_p_pid;  //pid
    PidTypeDef pitch_down_auto_s_pid;  //pid

    PidTypeDef pitch_up_auto_p_pid;  //pid
    PidTypeDef pitch_up_auto_s_pid;  //pid
    #elif (PITCH_PID_MODE == 2)
    PidTypeDef pitch_p_pid;  //pid
    PidTypeDef pitch_s_pid;  //pid

    PidTypeDef pitch_auto_p_pid;  //pid
    PidTypeDef pitch_auto_s_pid;  //pid
    #endif
    float accel_up;
    float accel_down;

    int8_t init_flag;    //初始化成功标志

    float Auto_record_location;                      // 实际位置


    first_order_filter_type_t LowFilt_Pitch_Data;    //P轴低通滤波器
    sliding_mean_filter_type_t Slidmean_Pitch_Data;  //P轴滑动滤波器

    first_order_filter_type_t LowFilt_auto_pitch;    //自瞄P轴低通滤波器
    sliding_mean_filter_type_t Slidmean_auto_pitch;  //自瞄P轴滑动滤波器

    int16_t filt_output; //P轴滤波值

    int16_t output;

} gimbal_pitch_control_t;


typedef struct  //申明yaw轴电机变量
{
    const motor_measure_t *yaw_motor_measure;

    PidTypeDef yaw_p_pid;  //pid
    PidTypeDef yaw_s_pid;  //pid

    PidTypeDef yaw_auto_p_pid;  //pid
    PidTypeDef yaw_auto_s_pid;  //pid


    uint8_t init_flag;           //Y轴初始化成功标志
    int8_t photoelectric_zero;  //Y轴中值光电标志

//	int16_t chassis_different_angle;  //云台底盘差角

    float angle;              //云台当前角度
    float last_angle;         //云台保存角度

    int16_t filt_output;  //Y轴滤波值

    int16_t output;

} gimbal_yaw_control_t;



typedef struct
{
    const RC_ctrl_t *gimbal_RC;     //底盘使用的遥控器指针
    Vision_Auto_Data_t *auto_c;     //申明自瞄变量

    VisionStatus_E VisionStatus;    // 敌人出现状态
    const Fire_task_t *Fire_task_control;
   
    gimbal_behaviour_e gimbal_behaviour;

    gimbal_pitch_control_t pitch_c;   //申明pitch轴电机变量
    gimbal_yaw_control_t yaw_c;       //申明yaw轴电机变量
    motor_measure_t motor_chassis[4];

    uint8_t Gimbal_all_flag;  //全部初始化完成标志

} gimbal_control_t;




/********************************* 函数声明 ***********************************/

extern void GIMBAL_TASK(void const * argument);
extern gimbal_behaviour_e Return_gimbal_mode(void);
extern void Gimbal_Manual_Work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3);
extern void Gimbal_Automatic_Work(gimbal_control_t *gimbal_automatic_work_f);


#endif
