/**
******************************************************************************
* @file       chassis_behaviour.c/h
* @brief      ����״̬����
******************************************************************************
*/

#include "chassis_behaviour.h"
#include "SysInit.h"


/**************** �������� *******************/ 
static void Chassis_Stop(chassis_control_t *Chassis_Stop_f);                              // ��������״̬����
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f);                              // �����Զ�״̬����
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f);                      // ������λ״̬����
static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f);            // ����ң��״̬����

static uint8_t Get_Laser_Back(void);
static uint8_t Get_Laser_Forward(void);

/********************************************/

/****************** ���� *********************/

 float Chassis_ch0 = 0.0f, Chassis_ch1 = 0.0f, Chassis_ch2 = 0.0f; //���̵���ܿ���(ch0����ң��ģʽ�Ŀ�������ch2�����Զ�ģʽ�Ŀ�����)
static uint16_t STANDBY_chassis_error_count = 0;										// ң��ͨ��ch[2],����ʱ����ֲ��˿��ض��ݴ���


 //����ң��ѡ��״̬��
 Chassis_mode_e chassis_remote_control_Table[RC_SW1_lift][RC_SW2_right] = 
{           /*����*/                     /* ���� */                          /* ���� */
  /*����*/  {CHASSIS_AUTO,              CHASSIS_STANDBY,             CHASSIS_REMOTECONTROL},
  /*����*/  {CHASSIS_REMOTECONTROL,     CHASSIS_STANDBY,             CHASSIS_REMOTECONTROL},
  /*����*/  {CHASSIS_AUTO,              CHASSIS_STANDBY,             CHASSIS_REMOTECONTROL}
};           


/*********************************************/


/**
  * @brief          ����ģʽ����
  * @param[in]      *chassis_control_t���������ṹ��
  * @retval         none
  */

void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f)
{
    const char rc_sw1_lift  		        = (chassis_behaviour_f ->chassis_RC ->rc.s[1] - 1);           // ң�ز���ֵ��1
		const char rc_sw2_right 		        = (chassis_behaviour_f ->chassis_RC ->rc.s[0] - 1);           
    chassis_behaviour_f ->chassis_mode  = chassis_remote_control_Table[rc_sw1_lift][rc_sw2_right];    // ��ȡ��ǰ����״̬

    /* ��ȡң����ֵ��ѡ��ģʽ */
    switch (chassis_behaviour_f -> chassis_mode)
    {
      /* ֮ǰΪ����״̬ */
			case CHASSIS_STOP:
      {
        Remote_reload(); //ҡ��������
        Chassis_Stop(chassis_behaviour_f);
				break;
      }
			/* ֮ǰΪ����״̬ */
      case CHASSIS_STANDBY:
      {
				if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)   // ң��ͨ��ch[2],����ʱ����ֲ��˿��ض��ݴ��󣬵���s1��s2��ͻȻ���2
				{			
         chassis_behaviour_f -> chassis_mode = CHASSIS_INITIALIZE;  //״̬����Ϊ��ʼ��
         Remote_reload(); 
				}
				else STANDBY_chassis_error_count++;												//ҡ��������
         break;                                           
      }
			/*  �Զ�ģʽ */
      case CHASSIS_AUTO:
      {
        Chassis_Auto(chassis_behaviour_f);
        break;
      }
      /* ң��ģʽ */
      case CHASSIS_REMOTECONTROL:
      {
        Chassis_RemoteControl(chassis_behaviour_f);
        break;
      }
      /* ��λģʽ */
      case CHASSIS_BLOCKING:
      {
        Chassis_Blocking(chassis_behaviour_f);
        break;
      }

			default:
			{
				chassis_behaviour_f->chassis_mode = CHASSIS_STANDBY; //�������ģʽ
				break;
			}
    }

			
    // ����ֵ����
		if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)
		{
			chassis_set_remote(chassis_behaviour_f,Chassis_ch0,Chassis_ch1, Chassis_ch2);
		}
		if(chassis_behaviour_f->chassis_mode != CHASSIS_STANDBY)
		STANDBY_chassis_error_count = 0;

} 


/**
  * @brief          ��������
  * @param[in]      *Chassis_Stop_f���������ṹ��
  * @retval         none
  */
static void Chassis_Stop(chassis_control_t *Chassis_Stop_f)
{
    //*������
    Chassis_Stop_f->chassis_motor[0].output = 0;
    Chassis_Stop_f->chassis_motor[1].output = 0;
    Chassis_Stop_f->chassis_motor[2].output = 0;
    Chassis_Stop_f->chassis_motor[3].output = 0;

    Chassis_ch0 = Chassis_ch1 = Chassis_ch2 = 0;
/*     Fire_mode = STOP_FIRE;
   Friction_wheel_mode = STOP_SHOOT;  */
}


/**
  * @brief          �����Զ�
  * @param[in]      *Chassis_Auto_f���������ṹ��
  * @retval         none
  */
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f)
{
    Chassis_ch2 = -CHASSIS_AUTO_SPPED;

}



/**
  * @brief          ������λ
* @param[in]      *Chassis_Blocking_f:  �������ṹ��
  * @retval         none
  */
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f)
{

}



/**
  * @brief          ����ң��
  * @param[in]      *Chassis_Independent_f���������ṹ��
  * @retval         none
  */
uint8_t back = 0 ,forward = 0 ;

 static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f)
{
    Chassis_ch0 = Chassis_RemoteControl_f->chassis_RC->rc.ch[0];
		back 		= Get_Laser_Back();
	  forward = Get_Laser_Forward();
}

//��ȡ���⴫������ֵ
//�������ж���Ϊ0, û����Ϊ1
//����������

static uint8_t Get_Laser_Back(void)
{
		static int8_t b;
		b = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
		
		return b;
}

static uint8_t Get_Laser_Forward(void)
{
		static int8_t a;
		a = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
	
		return a;
}

