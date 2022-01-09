/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       fire_task.c/h
  * @brief      火控任务
  * @note       合并版本
  * @history
  *
  @verbatim   v1.0
  ==============================================================================
  * 发射模式：1.高射速模式（即用到了射速三个等级中的最高射速来给摩擦轮）（精确度高。打风车时或者远程时用到）
  *         2.低射速模式（默认用最低射速限制15m/s速度，保证高射频）（精确度低。2.5m范围内近战用）
  *
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/

#include "Task_Fire.h"
#include "SysInit.h"

/******************* 变量 *********************/


/* 全局声明 */
Fire_task_t Fire_param;
/* 拨弹电机堵转计数 */
static uint16_t loading_A_stop_count = 0;
static uint16_t loading_B_stop_count = 0;
/* 拨弹电机堵转次数 */
static uint16_t loading_A_time = 0;
static uint16_t loading_B_time = 0;
/* 拨弹电机电流输出值 */
static int16_t loading_A_speed = LOADING_SPEED;
static int16_t loading_B_speed = -LOADING_SPEED;

/****************** 函数声明 ************************/
static void snail_motor_pwm(uint32_t snial_pwm);
static void Fire_param_init(void);
static void trigger_motor_turn_back(void);
static void Fire_Control(void);

//返回火控控制变量，通过指针传递方式传递信息
const Fire_task_t *get_Fire_control_point(void)
{
	return &Fire_param;
}

void Fire_Task(void *pvParameters)
{
	vTaskDelay(FIRE_TASK_INIT_TIME); //加载时间

	Fire_param_init(); // 初始化

	while (1)
	{

		/*  心跳任务 */
		LEDE4(0);

		snail_motor_pwm(1500);
		/* 允许控制 */
		if (Return_gimbal_mode() != GIMBAL_STANDBY && Return_gimbal_mode() != GIMBAL_STOP)
		{
			Fire_Control();
		}

		else // 不允许控制
		{
			snail_motor_pwm(FRICTION_SHOOT_STOP); // 摩擦轮停止

			Fire_param.GDA_output = 0;
			Fire_param.GDB_output = 0;
		}

		vTaskDelay(FIRE_CONTROL_TIME_MS);
	}
}

/**
 * @brief          热量限制检测
 * @param[in]      none
 * @retval         none
 * @attention
 */
void shoot_hot_limit(void)
{
#if (hot_limit == 1)
	uint16_t Residual_power_percentage;

	Fire_param.hot_max = get_shooter_cooling_limit();	 //机器人 1 号 17mm 枪口热量上限
	Fire_param.hot_current = get_shooter_cooling_heat(); // 1 号 17mm 枪口热量
	//		hot_current = get_shooter_cooling_heat();

	Residual_power_percentage = (Fire_param.hot_max - Fire_param.hot_current);

	if (Residual_power_percentage > 0 && Residual_power_percentage < 60)
	{
		//做热量限制
		Fire_param.GD_speed_gain = (float)(Residual_power_percentage / 60.0f) * (Residual_power_percentage / 60.0f);
	}

	else
	{
		//不做热量限制
		Fire_param.GD_speed_gain = 1.0f;
	}

#else
	Fire_param.GD_speed_gain = 1.0f; // 热量限制增益
#endif
}

/**
 * @brief          snail电机占空比控制
 * @param[in]      snial_pwm ：输入重转载值
 * @retval         none
 * @attention
 */
static void snail_motor_pwm(uint32_t snial_pwm)
{
#if (REVERES_LIGHT_COUPLING == 1)
	if ((snial_pwm >= 600) && (snial_pwm <= 1600))
		PWM_Shoot_Upper_Left = PWM_Shoot_Lower_Left = PWM_Shoot_Upper_Right = PWM_Shoot_Lower_Right = 2000 - snial_pwm;
#else
	if ((snial_pwm >= 600) && (snial_pwm) <= 1600)
		PWM_Shoot_Upper_Left = PWM_Shoot_Lower_Left = PWM_Shoot_Upper_Right = PWM_Shoot_Lower_Right = snial_pwm;
#endif
}

/**
 * @brief          参数初始化
 * @param[in]      none
 * @retval         none
 * @attention
 */
static void Fire_param_init(void)
{
	/* 看C615电调手册：
	 *       最大兼容控制信号频率: 500Hz  （目前是476Hz 防止浮动
	 *       控制信号行程: 400~2200微秒 （0.4ms~2.2ms）
	 *       默认输出PWM频率: 16kHz （和SNAIL电机手册里面的默认输入PWM频率16kHz对应）
	 */
	/******************** 开启PWM *******************/
	/*
	if (Get_appinit_status() == GIMBAL_APP)
	{
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	}
	*/


	//获取拨弹电机指针
	Fire_param.fire_motorA_measure = Get_Fire_MotorA_Measure_Point();
	Fire_param.fire_motorB_measure = Get_Fire_MotorB_Measure_Point();

	//射弹模式默认为停火
	Fire_param.fire_workstatus = STOP_FIRE;
	//摩擦轮模式停止模式
	Fire_param.Friction_workstatus = STOP_SHOOT;

	pid_init(&Fire_param.fire_s_pid, FIRE_S_P, FIRE_S_I, FIRE_S_D, 0, 0);
	pid_init(&Fire_param.fire_p_pid, FIRE_P_P, FIRE_P_I, FIRE_P_D, 0, 0);

	Fire_param.GDA_output = 0;
	Fire_param.GDB_output = 0;
	Fire_param.shoot_speed = 0;
	snail_motor_pwm(FRICTION_SHOOT_STOP);
}

/**
 * @brief          火控系统控制
 * @param[in]      none
 * @retval         none
 * @attention
 */
static void Fire_Control(void)
{
	/* 热量限制 */
	shoot_hot_limit();

	/* 获取摩擦轮模式（射速） */
	Fire_param.Friction_workstatus = Return_Friction_Wheel_Mode();
	/* 获取火控模式（是否开火） */
	Fire_param.fire_workstatus = Return_Fire_Mode();

	/* 摩擦轮控制 */
	if (Fire_param.Friction_workstatus == LOW_SPEED)
	{
		snail_motor_pwm(FRICTION_ONE_SHOOT_SPEED);
	}

	else if (Fire_param.Friction_workstatus == HIGH_SPEED)
	{
		snail_motor_pwm(FRICTION_THREE_SHOOT_SPEED);
	}

	else if (Fire_param.Friction_workstatus == STOP_SHOOT)
	{
		snail_motor_pwm(FRICTION_SHOOT_STOP);
	}

	else
	{
		snail_motor_pwm(FRICTION_SHOOT_STOP);
	}

	/* 供弹电机控制 GD=供弹 */
	if (Fire_param.fire_workstatus == STOP_FIRE)
	{
		Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
		Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}

	else if (Fire_param.fire_workstatus == FIRE)
	{
		/* 堵转处理 */
		trigger_motor_turn_back();

		Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, (loading_A_speed * Fire_param.GD_speed_gain), Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
		Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, (loading_B_speed * Fire_param.GD_speed_gain), Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}

	/* 退弹 */
	else if (Fire_param.fire_workstatus == BACK)
	{
		Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, -LOADING_SPEED, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
		Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, LOADING_SPEED, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}

	else
	{
		Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
		Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}
}

/**
 * @brief      堵转处理
 * @param[in]  void
 * @retval     void
 * @attention
 */
static void trigger_motor_turn_back(void)
{
	/*  当拨弹电机转速低于一定值 */
	if (Fire_param.fire_motorA_measure->speed >= (LOADING_SPEED + 700) && Fire_param.fire_motorA_measure->speed <= (-(LOADING_SPEED + 700)))
	{
		/*  拨转电机堵转计数值 */
		loading_A_stop_count++;

		/*  堵转计数值高于一定数 ，记录堵转次数 +1 */
		if (loading_A_stop_count >= LOADING_STOP_TIMES)
		{
			loading_A_time++;
			loading_A_stop_count = 0;
		}
	}

	if (Fire_param.fire_motorB_measure->speed >= (LOADING_SPEED + 700) && Fire_param.fire_motorB_measure->speed <= (-(LOADING_SPEED + 700)))
	{
		loading_B_stop_count++;

		if (loading_B_stop_count >= LOADING_STOP_TIMES)
		{
			loading_B_time++;
			loading_B_stop_count = 0;
		}
	}

	if (loading_A_time % 2 != 0) // 防止一直重复反转
		loading_A_speed = (-LOADING_SPEED);
	else
		loading_A_speed = LOADING_SPEED;

	if (loading_B_time % 2 != 0)
		loading_B_speed = (LOADING_SPEED);
	else
		loading_B_speed = (-LOADING_SPEED);
}
