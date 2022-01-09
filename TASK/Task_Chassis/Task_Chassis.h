/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       chassis_task.c/h
  * @brief      ��ɵ��̿�������
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/
#ifndef TASK_CHASSIS_H
#define TASK_CHASSIS_H


#include "RemoteControl.h"
#include "CAN_1_Receive.h"
#include "pid.h"
#include "maths.h"

/******************************  �궨�� ****************************************/
/*OS�������������Լ�����ʱ��*/
#define CHASSIS_TASK_INIT_TIME 5
#define CHASSIS_CONTROL_TIME   2

/**********************��ͨ�˲�����**********************/
#define CHASSIS_FIRST_ORDER_FILTER_K  0.0510f  //0.0110f ԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���  |  0.26f  |  0.0097f

/**********************���̵��pid����**********************/

//���̵���ٶȲ���
#define CHASSIS_MOTOR_PID_Kp    8.0f
#define CHASSIS_MOTOR_PID_Ki    2.0f
#define CHASSIS_MOTOR_PID_Kd    0.0f

//λ�û�����
#define CHASSIS_LOCATION_PID_P  100.0f
#define CHASSIS_LOCATION_PID_I  10.0f
#define CHASSIS_LOCATION_PID_D  10.8f

// ����
#define	GOFORWARD	 1				//�Զ�ģʽ�µ���ǰ
#define	GOBACK		 0				//�Զ�ģʽ�µ����

// ���⴫����
#define HAVE_THING 1		//�������ж���0     
#define NO_THING   0		//������û����1


#define SPEED_TIME_FLAG				430	//510			//����ʱ��
#define SPEED_MODE					4//���̱��٣�1 -> �����٣� 2 -> ����  ; 3 -> ���� + ����1��������� ;4 -> ����2���Ȳ����У�
/**********************�˶����ٶ�����**********************/
#define STRAIGHT_ACCELERAD        3.5f      //ֱ�е��̼��ٶ�����
#define TRANSLATION_ACCELERAD     5.5f      //ƽ�Ƶ��̼��ٶ�����


#define CHASSIS_AUTO_SPPED				4000			 //21����Ϊ7000			
#define CHASSIS_BLOCKING_SPPED		    6700 //��λ�ٶ� ���������ٶ� 7000  
#define CHASSIS_AUTO_SLOW_SPPED			0    // �������� �ٶȼ���		5.17   3000->0  5.25

/***********************************  �ṹ�� *************************************/

/*���̵������*/
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


/*����ģʽ*/
typedef enum
{
    CHASSIS_STOP,       //ֹͣ
    CHASSIS_INITIALIZE, //��ʼ����
    CHASSIS_STANDBY,    //����

    //CHASSIS_WORKING,   //����

    CHASSIS_FOLLOW,    //����
    CHASSIS_NO_FOLLOW, //������

    CHASSIS_TWIST_WAIST, //Ť��
    CHASSIS_ROTATION,    //С����
    CHASSIS_BATTERY,     //��̨ģʽ

    CHASSIS_REMOTECONTROL,//ң��ģʽ
    CHASSIS_AUTO,				   //�Զ�ģʽ
    CHASSIS_BLOCKING,      //��λģʽ
    CHASSIS_FIXATION			// ���̶̹�ģʽ
} Chassis_mode_e;


/*������ɫ*/
typedef enum
{
    Enemy_color_red = 0,    //������ɫΪ��ɫ
    Enemy_color_blue		//������ɫΪ��ɫ

} Enemy_color_e;


/*�����������ݽṹ��*/
typedef struct
{
   const  RC_ctrl_t *chassis_RC;             //����ʹ�õ�ң����ָ��
	
//    const gimbal_yaw_receive_t *gimbal_re_data; //��̨�崦�����ݣ�������can2��������̰壬���̰��������yaw����
//    const Fire_task_t *Fire_task_control;
//	
    motor_measure_t *yaw_motor_measure;      //can1ֱ�ӽ��յ�yaw������ �����ڷ�����̨����
    motor_measure_t *chassis_motor_measure;

//    PowerLimit_t Chassis_PowerLimit; //���̹������ƽṹ��
    Motor_t chassis_motor[4];        //���̵������(�������ͳһ�ṹ��ָ��)
    PidTypeDef chassis_speed_pid[4]; //�����ƶ�pid
    PidTypeDef chassis_location_pid; //����λ�û�pid

    Chassis_mode_e chassis_mode; //���̿���״̬��

    first_order_filter_type_t LowFilt_chassis_vx; //��ͨ�˲���
    first_order_filter_type_t LowFilt_chassis_vy; //��ͨ�˲���

    fp32 speed_x_set; //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
    fp32 speed_y_set; //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s

    fp32 chassis_gimbal_angel; //��������̨�ĽǶ�

    uint8_t sign;								//ǰ���߱�־
    uint8_t sign_last;						//����ǰ���߱�־

} chassis_control_t;



/********************************* �������� ***********************************/

extern  void Chassis_Task(void const * argument);
extern  void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2);
extern  void Chassis_Task_OFF(uint8_t options);
extern uint8_t  automatic_Enemy_color(void);


#endif





