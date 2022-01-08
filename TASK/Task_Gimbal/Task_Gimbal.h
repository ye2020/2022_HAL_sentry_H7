/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       gimbal_task.c/h
  * @brief      ��̨����
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/
	
#ifndef TASK_GIMBAL_H
#define TASK_GIMBAL_H



#include "RemoteControl.h"
#include "CAN_1_Receive.h"
#include "pid.h"
#include "maths.h"
#include "automatic_strike.h"
#include "Task_Fire.h"

/******************************  �궨�� ****************************************/


#define PITCH_PID_MODE            2 // ģʽ1�� �ֶ�pid ��pitch������pid�ֿ� �� ģʽ2 �ϲ�pid


/*********************** ң���ٶ����� *******************************/
#define RC_YAW_SPEED          0.0026f   //ң����yaw���ٶ�����
#define RC_PITCH_SPEED        0.0026f   //ң����pitch���ٶ����� 0.0026

/**********************��̨pitch��yaw�Ƕ�����**********************/
#define PITCH_ANGLE_LIMIT_UP    30
#define PITCH_ANGLE_LIMIT_DOWN  (-15)
#define YAW_ANGLE_LIMIT         15

/**********************pitch��yaw���������**********************/
#define YAW_OUTPUT_LIMIT         11000
#define YAW_INIT_OUTPUT_LIMIT    8000
#define PITCH_OUTPUT_LIMIT       8000
#define PITCH_INIT_OUTPUT_LIMIT  5000

/**********************��ͨ�˲�����**********************/
#define Gimbal_Pitch_Fir_Ord_Low_Fil_Param  0.0884f


/*OS�������������Լ�����ʱ��*/
#define GIMBAL_TASK_INIT_TIME 	5
#define GIMBAL_CONTROL_TIME_MS  2

/********************** P��pid���� **********************/

#define GIMBAL_P_YAW_P  130.0f   //Yλ�û�    150     62    130
#define GIMBAL_P_YAW_I  0.0f
#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_S_YAW_P  9.0f     //Y�ٶȻ�     8      10     9
#define GIMBAL_S_YAW_I  0.0f
#define GIMBAL_S_YAW_D  0.0f     //2                   0


#if (PITCH_PID_MODE == 1)

    #define GIMBAL_UP_P_PITCH_P  40.5f   //Pλ�û�  53   410    160       58  41.5
    #define GIMBAL_UP_P_PITCH_I  0.0f     //8f
    #define GIMBAL_UP_P_PITCH_D  3.8f     //0.0f       360       100     100

    #define GIMBAL_UP_S_PITCH_P  20.5f   //P�ٶȻ�(��Ҫ��i)       1.5     9.5
    #define GIMBAL_UP_S_PITCH_I  0.0f
    #define GIMBAL_UP_S_PITCH_D  3.8f

    #define GIMBAL_DOWN_P_PITCH_P  38.0f   //Pλ�û�  53   410    160       58
    #define GIMBAL_DOWN_P_PITCH_I  0.0f     //8f
    #define GIMBAL_DOWN_P_PITCH_D  3.8f     //0.0f       360       100     100

    #define GIMBAL_DOWN_S_PITCH_P  10.5f   //P�ٶȻ�(��Ҫ��i)       1.5     9.5
    #define GIMBAL_DOWN_S_PITCH_I  0.0f
    #define GIMBAL_DOWN_S_PITCH_D  3.8f

#elif (PITCH_PID_MODE == 2)


    #define GIMBAL_P_PITCH_P  60.0f         //Pλ�û�  70
    #define GIMBAL_P_PITCH_I 	1.5f		// 1.1f
    #define GIMBAL_P_PITCH_D  75.0f         //0.0f   75

    #define GIMBAL_S_PITCH_P  10.5f     //P�ٶȻ�(��Ҫ��i) 1.5     9.5
    #define GIMBAL_S_PITCH_I  0.0f
    #define GIMBAL_S_PITCH_D  6.0f	    //6.0f    //3.5  4

#endif

/*********�Ӿ�PY������pid��������***************/

#if (PITCH_PID_MODE == 1)
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_P 57.0f    //P�Զ�λ�û�  530.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_D 3.0f    //200.0f

    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_P 25.0f     //P�Զ��ٶȻ�
    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_D 3.0f

    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_P 20.0f    //P�Զ�λ�û�  530.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_D 0.0f    //200.0f

    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_P 10.0f     //P�Զ��ٶȻ�
    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_D 0.0f


#elif (PITCH_PID_MODE == 2)
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_P 36.0f    //P�Զ�λ�û�  30.0f |530.0f |35 |  30
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_I 0.1f
    #define GIMBAL_AUTO_INDUSTRY_P_PITCH_D 9.0f     //200.0f    9.0

    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_P 10.0f     //P�Զ��ٶȻ�  12  8
    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_S_PITCH_D 0.0f      //5
#endif

    #define GIMBAL_AUTO_INDUSTRY_P_YAW_P 100.0f      //Y�Զ�λ�û� 63.0f |60.0f |170
    #define GIMBAL_AUTO_INDUSTRY_P_YAW_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_P_YAW_D 15.0f

    #define GIMBAL_AUTO_INDUSTRY_S_YAW_P 15.0f       //Y�Զ��ٶȻ� 12
    #define GIMBAL_AUTO_INDUSTRY_S_YAW_I 0.0f
    #define GIMBAL_AUTO_INDUSTRY_S_YAW_D 1.0f



/***********************************  �ṹ�� *************************************/



typedef enum
{
    GIMBAL_STOP,         //ֹͣ
    GIMBAL_INITIALIZE,   //��ʼ��״̬
    GIMBAL_STANDBY,      //����״̬

    GIMBAL_WORKING,      //��ʼ������λ,��ʱûʲô��
    GIMBAL_REMOTECONTROL,       //�ֶ�״̬

    GIMBAL_REMOTECONTROL_STOP_SHOOT,       // ң��ֹͣ���״̬
    GIMBAL_REMOTECONTROL_LOW_SPEED,        // ң�ص�����״̬
    GIMBAL_REMOTECONTROL_HIGH_SPEED,       // ң�ظ�����״̬

    GIMBAL_AUTOCONTROL,  //����ģʽ
    GIMBAL_PATROl,       //Ѳ��״̬
    GIMBAL_AUTOATTACK,   //����״̬

    GIMBAL_DOUBLE_GIMBAL,//˫��̨״̬�����ޣ�֮������У�

} gimbal_behaviour_e;


typedef enum
{
	Enemy_Appear=0,		        //���ֵ���
	Enemy_Disappear=1,			//������ʧ
}VisionStatus_E;		        //���Զ�����ģʽ����ʹ��



typedef struct  //����pitch��������
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

    int8_t init_flag;    //��ʼ���ɹ���־

    float Auto_record_location;                      // ʵ��λ��


    first_order_filter_type_t LowFilt_Pitch_Data;    //P���ͨ�˲���
    sliding_mean_filter_type_t Slidmean_Pitch_Data;  //P�Ử���˲���

    first_order_filter_type_t LowFilt_auto_pitch;    //����P���ͨ�˲���
    sliding_mean_filter_type_t Slidmean_auto_pitch;  //����P�Ử���˲���

    int16_t filt_output; //P���˲�ֵ

    int16_t output;

} gimbal_pitch_control_t;


typedef struct  //����yaw��������
{
    const motor_measure_t *yaw_motor_measure;

    PidTypeDef yaw_p_pid;  //pid
    PidTypeDef yaw_s_pid;  //pid

    PidTypeDef yaw_auto_p_pid;  //pid
    PidTypeDef yaw_auto_s_pid;  //pid


    uint8_t init_flag;           //Y���ʼ���ɹ���־
    int8_t photoelectric_zero;  //Y����ֵ����־

//	int16_t chassis_different_angle;  //��̨���̲��

    float angle;              //��̨��ǰ�Ƕ�
    float last_angle;         //��̨����Ƕ�

    int16_t filt_output;  //Y���˲�ֵ

    int16_t output;

} gimbal_yaw_control_t;



typedef struct
{
    const RC_ctrl_t *gimbal_RC;     //����ʹ�õ�ң����ָ��
    Vision_Auto_Data_t *auto_c;     //�����������

    VisionStatus_E VisionStatus;    // ���˳���״̬
    const Fire_task_t *Fire_task_control;
   
    gimbal_behaviour_e gimbal_behaviour;

    gimbal_pitch_control_t pitch_c;   //����pitch��������
    gimbal_yaw_control_t yaw_c;       //����yaw��������
    motor_measure_t motor_chassis[4];

    uint8_t Gimbal_all_flag;  //ȫ����ʼ����ɱ�־

} gimbal_control_t;




/********************************* �������� ***********************************/

extern void GIMBAL_TASK(void const * argument);
extern gimbal_behaviour_e Return_gimbal_mode(void);
extern void Gimbal_Manual_Work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3);
extern void Gimbal_Automatic_Work(gimbal_control_t *gimbal_automatic_work_f);


#endif
