/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       fire_task.c/h
  * @brief      �������
  * @note       �ϲ��汾
  * @history
  *
  @verbatim   v1.0
  ==============================================================================
  * ����ģʽ��1.������ģʽ�����õ������������ȼ��е������������Ħ���֣�����ȷ�ȸߡ���糵ʱ����Զ��ʱ�õ���
  *         2.������ģʽ��Ĭ���������������15m/s�ٶȣ���֤����Ƶ������ȷ�ȵ͡�2.5m��Χ�ڽ�ս�ã�
  *
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/

#include "Task_Fire.h"
#include "SysInit.h"

/******************* ���� *********************/


/* ȫ������ */
Fire_task_t Fire_param;
/* ���������ת���� */
static uint16_t loading_A_stop_count = 0;
static uint16_t loading_B_stop_count = 0;
/* ���������ת���� */
static uint16_t loading_A_time = 0;
static uint16_t loading_B_time = 0;
/* ��������������ֵ */
static int16_t loading_A_speed = LOADING_SPEED;
static int16_t loading_B_speed = -LOADING_SPEED;

/****************** �������� ************************/
static void snail_motor_pwm(uint32_t snial_pwm);
static void Fire_param_init(void);
static void trigger_motor_turn_back(void);
static void Fire_Control(void);

//���ػ�ؿ��Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const Fire_task_t *get_Fire_control_point(void)
{
	return &Fire_param;
}

void Fire_Task(void *pvParameters)
{
	vTaskDelay(FIRE_TASK_INIT_TIME); //����ʱ��

	Fire_param_init(); // ��ʼ��

	while (1)
	{

		/*  �������� */
		LEDE4(0);

		snail_motor_pwm(1500);
		/* ������� */
		if (Return_gimbal_mode() != GIMBAL_STANDBY && Return_gimbal_mode() != GIMBAL_STOP)
		{
			Fire_Control();
		}

		else // ���������
		{
			snail_motor_pwm(FRICTION_SHOOT_STOP); // Ħ����ֹͣ

			Fire_param.GDA_output = 0;
			Fire_param.GDB_output = 0;
		}

		vTaskDelay(FIRE_CONTROL_TIME_MS);
	}
}

/**
 * @brief          �������Ƽ��
 * @param[in]      none
 * @retval         none
 * @attention
 */
void shoot_hot_limit(void)
{
#if (hot_limit == 1)
	uint16_t Residual_power_percentage;

	Fire_param.hot_max = get_shooter_cooling_limit();	 //������ 1 �� 17mm ǹ����������
	Fire_param.hot_current = get_shooter_cooling_heat(); // 1 �� 17mm ǹ������
	//		hot_current = get_shooter_cooling_heat();

	Residual_power_percentage = (Fire_param.hot_max - Fire_param.hot_current);

	if (Residual_power_percentage > 0 && Residual_power_percentage < 60)
	{
		//����������
		Fire_param.GD_speed_gain = (float)(Residual_power_percentage / 60.0f) * (Residual_power_percentage / 60.0f);
	}

	else
	{
		//������������
		Fire_param.GD_speed_gain = 1.0f;
	}

#else
	Fire_param.GD_speed_gain = 1.0f; // ������������
#endif
}

/**
 * @brief          snail���ռ�ձȿ���
 * @param[in]      snial_pwm ��������ת��ֵ
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
 * @brief          ������ʼ��
 * @param[in]      none
 * @retval         none
 * @attention
 */
static void Fire_param_init(void)
{
	/* ��C615����ֲ᣺
	 *       �����ݿ����ź�Ƶ��: 500Hz  ��Ŀǰ��476Hz ��ֹ����
	 *       �����ź��г�: 400~2200΢�� ��0.4ms~2.2ms��
	 *       Ĭ�����PWMƵ��: 16kHz ����SNAIL����ֲ������Ĭ������PWMƵ��16kHz��Ӧ��
	 */
	/******************** ����PWM *******************/
	/*
	if (Get_appinit_status() == GIMBAL_APP)
	{
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	}
	*/


	//��ȡ�������ָ��
	Fire_param.fire_motorA_measure = Get_Fire_MotorA_Measure_Point();
	Fire_param.fire_motorB_measure = Get_Fire_MotorB_Measure_Point();

	//�䵯ģʽĬ��Ϊͣ��
	Fire_param.fire_workstatus = STOP_FIRE;
	//Ħ����ģʽֹͣģʽ
	Fire_param.Friction_workstatus = STOP_SHOOT;

	pid_init(&Fire_param.fire_s_pid, FIRE_S_P, FIRE_S_I, FIRE_S_D, 0, 0);
	pid_init(&Fire_param.fire_p_pid, FIRE_P_P, FIRE_P_I, FIRE_P_D, 0, 0);

	Fire_param.GDA_output = 0;
	Fire_param.GDB_output = 0;
	Fire_param.shoot_speed = 0;
	snail_motor_pwm(FRICTION_SHOOT_STOP);
}

/**
 * @brief          ���ϵͳ����
 * @param[in]      none
 * @retval         none
 * @attention
 */
static void Fire_Control(void)
{
	/* �������� */
	shoot_hot_limit();

	/* ��ȡĦ����ģʽ�����٣� */
	Fire_param.Friction_workstatus = Return_Friction_Wheel_Mode();
	/* ��ȡ���ģʽ���Ƿ񿪻� */
	Fire_param.fire_workstatus = Return_Fire_Mode();

	/* Ħ���ֿ��� */
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

	/* ����������� GD=���� */
	if (Fire_param.fire_workstatus == STOP_FIRE)
	{
		Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
		Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, 0, Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}

	else if (Fire_param.fire_workstatus == FIRE)
	{
		/* ��ת���� */
		trigger_motor_turn_back();

		Fire_param.GDA_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, (loading_A_speed * Fire_param.GD_speed_gain), Fire_param.fire_motorA_measure->speed, M2006_MAX_OUTPUT_CURRENT);
		Fire_param.GDB_output = Rmmotor_Speed_control(&Fire_param.fire_s_pid, (loading_B_speed * Fire_param.GD_speed_gain), Fire_param.fire_motorB_measure->speed, M2006_MAX_OUTPUT_CURRENT);
	}

	/* �˵� */
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
 * @brief      ��ת����
 * @param[in]  void
 * @retval     void
 * @attention
 */
static void trigger_motor_turn_back(void)
{
	/*  ���������ת�ٵ���һ��ֵ */
	if (Fire_param.fire_motorA_measure->speed >= (LOADING_SPEED + 700) && Fire_param.fire_motorA_measure->speed <= (-(LOADING_SPEED + 700)))
	{
		/*  ��ת�����ת����ֵ */
		loading_A_stop_count++;

		/*  ��ת����ֵ����һ���� ����¼��ת���� +1 */
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

	if (loading_A_time % 2 != 0) // ��ֹһֱ�ظ���ת
		loading_A_speed = (-LOADING_SPEED);
	else
		loading_A_speed = LOADING_SPEED;

	if (loading_B_time % 2 != 0)
		loading_B_speed = (LOADING_SPEED);
	else
		loading_B_speed = (-LOADING_SPEED);
}
