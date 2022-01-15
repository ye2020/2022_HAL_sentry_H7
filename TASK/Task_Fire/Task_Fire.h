#ifndef TASK_FIRE_H
#define TASK_FIRE_H


#include "CAN_1_Receive.h"
#include "pid.h"
#include "RemoteControl.h"

/*OS控制任务周期以及启动时间*/
#define FIRE_TASK_INIT_TIME  5
#define FIRE_CONTROL_TIME_MS 2

#define REVERES_LIGHT_COUPLING  1 					// 是否使用反向光偶   1 -> 使用 0 -> 未使用
#define	LOADING_STOP_TIMES  700                     // 堵转计时


/*摩擦轮*/
#define PWM_Shoot_Upper_Left   TIM1->CCR1  //PE9
#define PWM_Shoot_Lower_Left   TIM1->CCR2	 //PE11
#define PWM_Shoot_Upper_Right  TIM8->CCR3  //PC8
#define PWM_Shoot_Lower_Right  TIM8->CCR4	 //PC9

/**********发弹系统速度设定**********/
//低射速：约15m/s （测试用）  高射速：约28m/s(比赛用)
#define FRICTION_THREE_SHOOT_SPEED   1210//(2000 - 1160) 	//30摩擦轮高速pwm			1205 -> 22.5m/s     (1243)
#define FRICTION_ONE_SHOOT_SPEED   	 1150///(2000- 1140)   //18摩擦轮高速pwm		原1173  13.5m/s			（1187）
#define FRICTION_SHOOT_STOP          900//(2000 - 880) 	   //0摩擦轮停止pwm
#define LOADING_SPEED        (-3300)    //供弹电机速度  //5.2 2000调整为2900


/**********拨弹电机速度环pid参数定义**********/
#define FIRE_S_P 12.0f   //供弹电机M2006速度环
#define FIRE_S_I 0.0f
#define FIRE_S_D 0.0f

#define FIRE_P_P 50.0f
#define FIRE_P_I 0.1f  //0 TODO
#define FIRE_P_D 0.1f


/*发弹模式*/
typedef enum
{
    FIRE,          //发射
    AUTO_FIRE,     //自动发射
    STOP_FIRE,     //停止发射
    BACK,          //退弹
    FIRE_ERROR,

} Fire_WorkStatus_e;


/*摩擦轮模式*/
typedef enum
{
    LOW_SPEED,
    HIGH_SPEED,
    STOP_SHOOT,
    SHOOT_ERROR,

} Shoot_WorkStatus_e;


/* 拨弹电机状态*/
typedef enum
{
	motor_A_blocking_mode,		// 拨弹电机A堵转模式
	motor_B_blocking_mode,		// 拨弹电机B堵转模式
	normal_mode,			// 正常模式

} Loading_motor_e;



/*火控结构体*/
typedef struct Fire_param
{
    const motor_measure_t *fire_motorA_measure;
	const motor_measure_t *fire_motorB_measure;
    const RC_ctrl_t *fire_RC;   //开火任务使用的遥控器指针
    PidTypeDef fire_s_pid;      //拨弹2006电机pid
	PidTypeDef fire_p_pid;			//拨弹2006电机位置pid

	
	Loading_motor_e Loading_motorStatus; //拨弹电机状态
    Fire_WorkStatus_e fire_workstatus;   //射弹模式
    Shoot_WorkStatus_e Friction_workstatus;  //摩擦轮模式

    int16_t GDA_output;          //供弹电机输出
	int16_t GDB_output;          //供弹电机输出
    uint8_t shoot_speed;        //当前设置射速
    uint8_t dead_mode;          //死亡模式开关
	uint16_t hot_max;						//枪口热量上限
	uint16_t hot_current;				//枪口热量
	float GD_speed_gain;				//热量限制 增益值

} Fire_task_t;

extern const Fire_task_t *get_Fire_control_point(void);

extern  void Fire_Task(void *pvParameters);

#endif
