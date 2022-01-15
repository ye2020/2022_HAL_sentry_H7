#ifndef TASK_FIRE_H
#define TASK_FIRE_H


#include "CAN_1_Receive.h"
#include "pid.h"
#include "RemoteControl.h"

/*OS�������������Լ�����ʱ��*/
#define FIRE_TASK_INIT_TIME  5
#define FIRE_CONTROL_TIME_MS 2

#define REVERES_LIGHT_COUPLING  1 					// �Ƿ�ʹ�÷����ż   1 -> ʹ�� 0 -> δʹ��
#define	LOADING_STOP_TIMES  700                     // ��ת��ʱ


/*Ħ����*/
#define PWM_Shoot_Upper_Left   TIM1->CCR1  //PE9
#define PWM_Shoot_Lower_Left   TIM1->CCR2	 //PE11
#define PWM_Shoot_Upper_Right  TIM8->CCR3  //PC8
#define PWM_Shoot_Lower_Right  TIM8->CCR4	 //PC9

/**********����ϵͳ�ٶ��趨**********/
//�����٣�Լ15m/s �������ã�  �����٣�Լ28m/s(������)
#define FRICTION_THREE_SHOOT_SPEED   1210//(2000 - 1160) 	//30Ħ���ָ���pwm			1205 -> 22.5m/s     (1243)
#define FRICTION_ONE_SHOOT_SPEED   	 1150///(2000- 1140)   //18Ħ���ָ���pwm		ԭ1173  13.5m/s			��1187��
#define FRICTION_SHOOT_STOP          900//(2000 - 880) 	   //0Ħ����ֹͣpwm
#define LOADING_SPEED        (-3300)    //��������ٶ�  //5.2 2000����Ϊ2900


/**********��������ٶȻ�pid��������**********/
#define FIRE_S_P 12.0f   //�������M2006�ٶȻ�
#define FIRE_S_I 0.0f
#define FIRE_S_D 0.0f

#define FIRE_P_P 50.0f
#define FIRE_P_I 0.1f  //0 TODO
#define FIRE_P_D 0.1f


/*����ģʽ*/
typedef enum
{
    FIRE,          //����
    AUTO_FIRE,     //�Զ�����
    STOP_FIRE,     //ֹͣ����
    BACK,          //�˵�
    FIRE_ERROR,

} Fire_WorkStatus_e;


/*Ħ����ģʽ*/
typedef enum
{
    LOW_SPEED,
    HIGH_SPEED,
    STOP_SHOOT,
    SHOOT_ERROR,

} Shoot_WorkStatus_e;


/* �������״̬*/
typedef enum
{
	motor_A_blocking_mode,		// �������A��תģʽ
	motor_B_blocking_mode,		// �������B��תģʽ
	normal_mode,			// ����ģʽ

} Loading_motor_e;



/*��ؽṹ��*/
typedef struct Fire_param
{
    const motor_measure_t *fire_motorA_measure;
	const motor_measure_t *fire_motorB_measure;
    const RC_ctrl_t *fire_RC;   //��������ʹ�õ�ң����ָ��
    PidTypeDef fire_s_pid;      //����2006���pid
	PidTypeDef fire_p_pid;			//����2006���λ��pid

	
	Loading_motor_e Loading_motorStatus; //�������״̬
    Fire_WorkStatus_e fire_workstatus;   //�䵯ģʽ
    Shoot_WorkStatus_e Friction_workstatus;  //Ħ����ģʽ

    int16_t GDA_output;          //����������
	int16_t GDB_output;          //����������
    uint8_t shoot_speed;        //��ǰ��������
    uint8_t dead_mode;          //����ģʽ����
	uint16_t hot_max;						//ǹ����������
	uint16_t hot_current;				//ǹ������
	float GD_speed_gain;				//�������� ����ֵ

} Fire_task_t;

extern const Fire_task_t *get_Fire_control_point(void);

extern  void Fire_Task(void *pvParameters);

#endif
