/**
******************************************************************************
* @file       gimbal_behaviour.c/h
* @brief      ��̨״̬����
******************************************************************************
*/


#include "gimbal_behaviour.h"
#include "SysInit.h"



/**************** ���� *******************/ 


// ����̨ң��״̬��
gimbal_behaviour_e gimbal_remote_control_Table[3][3] = 
{           /*����*/                     /* ���� */                          /* ���� */
  /*����*/  {GIMBAL_AUTOCONTROL,         GIMBAL_STANDBY,            GIMBAL_REMOTECONTROL_HIGH_SPEED   },
  /*����*/  {GIMBAL_STANDBY,             GIMBAL_STANDBY,            GIMBAL_REMOTECONTROL_STOP_SHOOT   },
  /*����*/  {GIMBAL_AUTOATTACK,          GIMBAL_STANDBY,            GIMBAL_REMOTECONTROL_LOW_SPEED    }
}; 

// ����̨ң��״̬��
gimbal_behaviour_e above_gimbal_remote_control_Table[3][3] = 
{           /*����*/                     /* ���� */                          /* ���� */
  /*����*/  {GIMBAL_AUTOCONTROL,         GIMBAL_REMOTECONTROL_HIGH_SPEED,       GIMBAL_STANDBY   },
  /*����*/  {GIMBAL_AUTOATTACK,    			 GIMBAL_STANDBY,                        GIMBAL_STANDBY   },
  /*����*/  {GIMBAL_STANDBY,      			 GIMBAL_REMOTECONTROL_STOP_SHOOT,       GIMBAL_STANDBY   }
};    



float Gimbal_ch2 = 0.0f, Gimbal_ch3 = 0.0f;          // ��̨����ܿ��� ( ch2Ϊywa���������ch3Ϊpitch��)
static Shoot_WorkStatus_e Friction_wheel_mode = STOP_SHOOT; // ������ٿ�����
static Fire_WorkStatus_e Fire_mode = STOP_FIRE;             // ��ؿ���ģʽ������

static uint16_t vision_status_count = 0;                    // ������ʧ����
static uint16_t STANDBY_error_count = 0;										// ң��ͨ��ch[2],����ʱ����ֲ��˿��ض��ݴ���


/**************** �������� *******************/ 

static void Gimbal_AutoControl(gimbal_control_t *gimbal_autocontrol_f);           //  ����ģʽ(����+Ѳ��)
static void Gimbal_Patrol(gimbal_control_t *gimbal_Patro_f);                      //  ��̨Ѳ��
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f);       //  ��̨ң��ģʽ



/**
  * @brief         ��̨״̬ѡ��
  * @param[in]      fir_gimbal_behaviour_f
  * @retval         none
  * @attention
  */
void Gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f)
{
    const char rc_sw1_lift    = (fir_gimbal_behaviour_f ->gimbal_RC ->rc.s[1] - 1);           // ң�ز���ֵ��1
		const char rc_sw2_right 	= (fir_gimbal_behaviour_f ->gimbal_RC ->rc.s[0] - 1); 
    
    if(Get_appinit_status() == GIMBAL_APP)          
    fir_gimbal_behaviour_f->gimbal_behaviour = gimbal_remote_control_Table[rc_sw1_lift][rc_sw2_right];
    else if(Get_appinit_status() == CHASSIS_APP)
    fir_gimbal_behaviour_f->gimbal_behaviour = above_gimbal_remote_control_Table[rc_sw1_lift][rc_sw2_right];

    switch (fir_gimbal_behaviour_f->gimbal_behaviour)
    {
      /*  ����ģʽ */
      case GIMBAL_AUTOCONTROL:
      {
        Gimbal_AutoControl(fir_gimbal_behaviour_f);
        break;
      }
      /*  ��ʼ�� */
      case GIMBAL_STANDBY:
      {
//				if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_error_count >= 5)   // ң��ͨ��ch[2],����ʱ����ֲ��˿��ض��ݴ��󣬵���s1��s2��ͻȻ���2
//				{																																					// δ����ԭ�򣬴˴�Ϊһ���α겻�α��Ĳ���
					Gimbal_Stop(fir_gimbal_behaviour_f);                       //ֹͣ
					Remote_reload();                                           //ҡ��������
//				}
//				else STANDBY_error_count++;
        break;
      }
      /* ����ģʽ */
      case GIMBAL_AUTOATTACK:
      {
        Gimbal_AutoControl(fir_gimbal_behaviour_f);
        Friction_wheel_mode = STOP_SHOOT;
			  Fire_mode = STOP_FIRE;
        break;
      }
      /*  �ֿص����� */
      case GIMBAL_REMOTECONTROL_LOW_SPEED:
      {
        Gimbal_RemoteControl(fir_gimbal_behaviour_f);									//ң�ؿ���
        Friction_wheel_mode = LOW_SPEED;
        break;
      }
      /*  �ֿ�ͣ�� */
      case GIMBAL_REMOTECONTROL_STOP_SHOOT:
      {
        Gimbal_RemoteControl(fir_gimbal_behaviour_f);									//ң�ؿ���
        Friction_wheel_mode = STOP_SHOOT;
        break;
      }
      /*  �ֿظ����� */
      case GIMBAL_REMOTECONTROL_HIGH_SPEED:
      {
        Gimbal_RemoteControl(fir_gimbal_behaviour_f);									//ң�ؿ���
        Friction_wheel_mode = HIGH_SPEED;
        break;
      }
      /*  ��̨���� */
      case GIMBAL_STOP:
      {
        Remote_reload();       //ҡ��������
        Gimbal_Stop(fir_gimbal_behaviour_f);
        break;
      }
      /* ң��ģʽ */
      case GIMBAL_REMOTECONTROL:
      {
        Gimbal_RemoteControl(fir_gimbal_behaviour_f);
        break;
      }

      case GIMBAL_PATROl:
      {
        Gimbal_Patrol(fir_gimbal_behaviour_f);
        break;
      }
    default:
      break;
    }
		
//		if(fir_gimbal_behaviour_f->gimbal_behaviour != GIMBAL_STANDBY)
//			STANDBY_error_count = 0;
}



/**
  * @brief          ��̨�Զ�״̬
  * @param[in]      gimbal_auto_f
  * @retval         �Ӿ����������� ��������̨�Զ�����
  * @attention
  */
void Gimbal_Auto(gimbal_control_t *gimbal_auto_f)
 {
    if(gimbal_auto_f ->VisionStatus == Enemy_Disappear)
    {
      /* ʵ��λ�� */
      gimbal_auto_f ->pitch_c.Auto_record_location = (gimbal_auto_f ->pitch_c.pitch_motor_measure ->actual_Position *360 /1024);
    }

    /*��̨�Ӿ�������*/    //(y�ᣬp�ỹû�ÿ�����)    
    gimbal_auto_f ->auto_c ->pitch_control_data = gimbal_auto_f->auto_c->auto_pitch_angle * 4.0f;
    gimbal_auto_f->auto_c->yaw_control_data     = gimbal_auto_f->auto_c->auto_yaw_angle * 4.0f;

    /* �޷� pitch��yaw��Ƕ����� */
    gimbal_auto_f->auto_c->pitch_control_data = float_limit(gimbal_auto_f->auto_c->pitch_control_data, 5.0f*PITCH_ANGLE_LIMIT_UP , 25.0f*PITCH_ANGLE_LIMIT_DOWN);
    gimbal_auto_f->auto_c->yaw_control_data   = float_limit(gimbal_auto_f->auto_c->yaw_control_data , YAW_ANGLE_LIMIT , -YAW_ANGLE_LIMIT);

    Gimbal_Automatic_Work(gimbal_auto_f);
 }


/**
  * @brief          ��̨����
  * @param[in]      gimbal_stop_f
  * @retval         none
  * @attention
  */
void Gimbal_Stop(gimbal_control_t *gimbal_stop_f)
{
    //������
    gimbal_stop_f->pitch_c.output = 0;
    gimbal_stop_f->yaw_c.output = 0;
    
    //���������
    gimbal_stop_f->auto_c->pitch_control_data = 0;
    gimbal_stop_f->auto_c->yaw_control_data = 0;

	  Friction_wheel_mode = STOP_SHOOT;  
    Fire_mode           = STOP_FIRE;

//    Gimbal_ch3 = 0.0f;
//    Gimbal_ch2 = 0.0f;

}


/**
  * @brief          ����ģʽ(����+Ѳ��)
  * @param[in]      gimbal_autocontrol_f
  * @retval         none
  * @attention
  */
static void Gimbal_AutoControl(gimbal_control_t *gimbal_autocontrol_f)
{
  /**************�жϵ��˳��ֻ���ʧ***************/
  {
    if(gimbal_autocontrol_f ->auto_c ->auto_pitch_angle == 0.0f && gimbal_autocontrol_f ->auto_c ->auto_yaw_angle == 0.0f)
    { 
       vision_status_count++;
    }
    else 
    {
       vision_status_count = 0;
       gimbal_autocontrol_f->VisionStatus = Enemy_Appear ;                 // ���˳���
    }

    if ( vision_status_count >= ENEMY_DISAPPEAR_TIMES)
    {
      vision_status_count = ENEMY_DISAPPEAR_TIMES;
      gimbal_autocontrol_f->VisionStatus = Enemy_Disappear;		            //������ʧ
    }
  }

  /********************** ״̬�ж���ѡ�� **********************/
  {
    if(gimbal_autocontrol_f->VisionStatus == Enemy_Disappear )      // ������ʧ
    {
      Gimbal_Patrol(gimbal_autocontrol_f);
      Fire_mode = STOP_FIRE;
    }

    if(gimbal_autocontrol_f->VisionStatus == Enemy_Appear)          // ���˳���
    {
      Gimbal_Auto(gimbal_autocontrol_f);
			Fire_mode = FIRE;
    }
  }

}

/**
  * @brief          ��̨Ѳ��
  * @param[in]      *gimbal_Patro_f
  * @retval         none
  */
static uint32_t yaw_direction = RIGHT ;                 // yaw  �᷽��
static uint32_t pitch_direction = PITCH_UP ;            // pitch�᷽��
float Auto_Yaw_Angle_Target = 0.0f;                     // yaw  �������
float Auto_Pitch_Angle_Target = 0.0f;                   // pitch�������
 static void Gimbal_Patrol(gimbal_control_t *gimbal_Patro_f)
{

#if yaw_angle_limit	
	/*------------------  yaw��Ƕ����� ȡ�����������yaw��360----------------------------*/
	
		if(gimbal_Patro_f->yaw_c.yaw_motor_measure->yaw_angle >= YAW_ANGLE_LIMIT )								
		{
				yaw_direction = LEFT;
		}
		else if(gimbal_Patro_f->yaw_c.yaw_motor_measure->yaw_angle <= -YAW_ANGLE_LIMIT)
		{
				yaw_direction = RIGHT;
		}
		
/*-----------------------------------------------------------------------------------------*/		
#endif		

		if(yaw_direction == LEFT)
		{
				Auto_Yaw_Angle_Target -=0.2f;   //������̨�Զ����ٶ�
		}
		if(yaw_direction == RIGHT)
		{
				Auto_Yaw_Angle_Target +=0.2f;
		}
		
		/*P��*/
		if(gimbal_Patro_f->pitch_c.pitch_motor_measure->pitch_angle >= PITCH_ANGLE_LIMIT_UP)
		{
				pitch_direction = PITCH_DOWN;
		}
		else if(gimbal_Patro_f->pitch_c.pitch_motor_measure->pitch_angle <= PITCH_ANGLE_LIMIT_DOWN)
		{
				pitch_direction = PITCH_UP;
		}
		
		if(pitch_direction == PITCH_UP)
		{
				Auto_Pitch_Angle_Target += 0.2f;
		}
		if(pitch_direction == PITCH_DOWN)
		{
				Auto_Pitch_Angle_Target -= 0.2f;
		}
		Gimbal_Manual_Work(gimbal_Patro_f , Auto_Yaw_Angle_Target , Auto_Pitch_Angle_Target);
}


/**
  * @brief          ��̨ң��ģʽ
  * @param[in]      gimbal_remotecontrol_f
  * @retval         none
  * @attention
  */
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f)
{	
    Gimbal_ch2 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[0]) * RC_YAW_SPEED * 0.2f;     //Y��λ�û����ۼ�   RC_YAW_SPEED
    Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f);                         //ѭ���޷���yaw�Ƕ�����     -180~180
    Gimbal_ch3 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[1]) * RC_PITCH_SPEED * 0.09f;  //P��λ�û����ۼ�  RC_PITCH_SPEED

    Gimbal_ch3 = float_limit(Gimbal_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);   //pitch�Ƕ�����   0 ~ -85  (�ڱ��ϸ�����)

    if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] >= 500)
    {
        Fire_mode = FIRE;        //������  ����
    }
    else if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] <= -500)
    {
        Fire_mode = BACK;       //������  �˵�����ת��
    }
    else
    {
        Fire_mode = STOP_FIRE;  //�����м�  ֹͣ����
    }

    Gimbal_Manual_Work(gimbal_remotecontrol_f , Gimbal_ch2 , Gimbal_ch3);
}




/**
  * @brief          ����Ħ����ģʽ
  * @param[in]      none
  * @retval         Friction_wheel_mode
  * @attention
  */
Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void)
{
//	LOW_SPEED,   //������
//	HIGH_SPEED,  //������
//	STOP_SHOOT,  //ֹͣ
    if (Friction_wheel_mode == LOW_SPEED)
        return LOW_SPEED;

    if (Friction_wheel_mode == HIGH_SPEED)
        return HIGH_SPEED;

    if (Friction_wheel_mode == STOP_SHOOT)
        return STOP_SHOOT;

    return SHOOT_ERROR;
}



/**
  * @brief          ���ط���ģʽ
  * @param[in]      none
  * @retval         Fire_mode
  * @attention
  */
Fire_WorkStatus_e Return_Fire_Mode(void)
{
//	FIRE,          //����
//	AUTO_FIRE,     //�Զ�����
//	STOP_FIRE,     //ֹͣ����
//	BACK,          //�˵�
    if (Fire_mode == FIRE)
        return FIRE;

    if (Fire_mode == AUTO_FIRE)
        return AUTO_FIRE;

    if (Fire_mode == STOP_FIRE)
        return STOP_FIRE;

    if (Fire_mode == BACK)
        return BACK;

    return FIRE_ERROR;
}

