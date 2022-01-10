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



 //����ң��ѡ��״̬��
 Chassis_mode_e chassis_remote_control_Table[RC_SW1_lift][RC_SW2_right] = 
{           /*����*/                     /* ���� */                          /* ���� */
  /*����*/  {CHASSIS_AUTO,              CHASSIS_REMOTECONTROL,             CHASSIS_REMOTECONTROL},
  /*����*/  {CHASSIS_AUTO,     			CHASSIS_STANDBY,             	   CHASSIS_REMOTECONTROL},
  /*����*/  {CHASSIS_AUTO,              CHASSIS_REMOTECONTROL,             CHASSIS_REMOTECONTROL}
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
//				if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)   // ң��ͨ��ch[2],����ʱ����ֲ��˿��ض��ݴ��󣬵���s1��s2��ͻȻ���2
				{			
         chassis_behaviour_f -> chassis_mode = CHASSIS_INITIALIZE;  //״̬����Ϊ��ʼ��
         Remote_reload(); 
				}
//				else STANDBY_chassis_error_count++;												//ҡ��������
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

			
//    // ����ֵ����
//	if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)
//	{
			chassis_set_remote(chassis_behaviour_f,Chassis_ch0,Chassis_ch1, Chassis_ch2);
//		}
//		if(chassis_behaviour_f->chassis_mode != CHASSIS_STANDBY)
//		STANDBY_chassis_error_count = 0;

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


/*********************** ���� *****************************/
static uint16_t outpost_HP = 0;													//ǰ��վѪ��

/**
  * @brief          �����Զ�
  * @param[in]      *Chassis_Auto_f���������ṹ��
  * @retval         none
  */
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f)
{

    if(automatic_Enemy_color() == Enemy_color_blue)       // ����Ϊ����
      outpost_HP = 	referee_red_outpost_HP();							// ��ȡ�췽ǰ��վѪ��

    else if (automatic_Enemy_color() == Enemy_color_red)  // �з�Ϊ�췽
      outpost_HP =  referee_blue_outpost_HP();            // ��ȡ����ǰ��վѪ��

    if(outpost_HP >= 10)                                  // ����ǰ��վѪ���ϸ�
      Chassis_ch2 = CHASSIS_AUTO_SLOW_SPPED;

     else if(outpost_HP < 10) 
     {
       if((get_Enemy_status() == Enemy_Disappear) && (get_Enemy_status_from_above() == Enemy_Disappear))          // ����δ����
       {
          Chassis_Blocking(Chassis_Auto_f);             // ��λģʽ�����Ʒ�����ٶȣ�

           if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))        // ��Եת�����ȼ�������λģʽ��
           {
              Chassis_Auto_f->sign = GOBACK;
           }
            if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
            {
              Chassis_Auto_f->sign = GOFORWARD;
            } 
       }

       else if((get_Enemy_status() == Enemy_Appear) || (get_Enemy_status_from_above() == Enemy_Appear))
       {
          Chassis_ch2 = CHASSIS_AUTO_SLOW_SPPED;

            if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))
            {
                Chassis_Auto_f->sign = GOBACK;
            }

            if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
            {
                Chassis_Auto_f->sign = GOFORWARD;
            }
       }
     }
}


static uint16_t outpost_HP;													//ǰ��վѪ��
 uint8_t  Enemy_color;												//������
static uint16_t 		  speed_status_count = 0;							// ����ʱ���ʱ
static uint16_t 		  speed_status_count2 = 0;						// ����ʱ���ʱ
static int32_t  chassis_register1 = 5000;//5000;//360;						//�ٶȼ�¼ֵ
static int32_t 	chassis_register2 = 0;							//�ٶȼ�¼ֵ
static uint16_t 			speed_status_flag = 0;							// ��Եת���ֹ��� (1 -> ���� 0 ��> ����)
static uint16_t 			forward_flag ,back_flag = 0;					// ���ģʽ1 ǿ��ת���־λ
#if (SPEED_MODE == 4)
uint16_t 		  speed_status_count3 = 0;									// ����ģʽ��ʱ���ʱ
uint16_t 			chassis_flag = 0;													//����ģʽ����־λ
uint16_t  		speed_time_back_flag = SPEED_TIME_FLAG;			//����ʱ��1
uint16_t  		speed_time_forward_flag = SPEED_TIME_FLAG;	//����ʱ��1
#endif	
#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
uint16_t 			last_remain_HP = 600; 											//�ϴ�ʣ��Ѫ��
uint16_t 			changed_HP;																	// �仯Ѫ��		
uint16_t      speed_status_count4 = 0;									// ģʽ5����ֵ
#endif	


/**
  * @brief          ������λ
* @param[in]      *Chassis_Blocking_f:  �������ṹ��
  * @retval       ��ps�� ���㷨�Ǻþ�֮ǰ��ǰ��ʱд�� û���κη�װ ��Ч������ Ŀǰ����ȥ��װ����������doge�� 
  */
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f)
{
#if   (SPEED_MODE == 1)										
            Chassis_ch2 = CHASSIS_AUTO_SPPED;
#elif (SPEED_MODE == 2 || SPEED_MODE == 3 || SPEED_MODE == 4)
						
						speed_status_count ++;
					
					if ((Get_Laser_Forward() == HAVE_THING) || (Get_Laser_Back() == HAVE_THING))	// �˶�����Ե �� ��־λ��1 -> һ��ʱ�����˶���������㷨Ӱ��
					{
							speed_status_flag = 1;
						
							speed_status_count 	= 0;
							speed_status_count2 = 0; 
						
							#if (SPEED_MODE == 4)																																												// ��ֹƵ��ת���µ������
							speed_status_count3 = 0; 
							#endif	
							#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
							speed_status_count4 = 0;
							#endif												
					}
					
					  if(speed_status_count > SPEED_TIME_FLAG)
					{	
						speed_status_flag = 0;																																// ��Եת���־λ
						
						if(speed_status_flag == 0 )
						{
							speed_status_count = 0;															  															//����ֵ����
							chassis_register1 = RNG_Get_RandomRange(4900,5200);//(4900,5000);											//����4000~7000�������(4900 5000)
							Chassis_ch2 = chassis_register1;																										//��ֵ�µ�����ٶ�ֵ
						}							
					}
						else
					{
						Chassis_ch2 = chassis_register1;												//�ٶ�ֵΪ��һ���������ֵ
					}
#endif					
#if (SPEED_MODE == 3)
					speed_status_count2 ++;
			
					if(speed_status_count2 > SPEED_TIME_FLAG)									// ��ʱʱ�䵽 �� ���������
					{
						speed_status_count2 =0;
						speed_status_flag = 0;																	
						chassis_register2 = RNG_Get_RandomRange(0,1);						// ����0~1�����
						
						if (chassis_register2 > 0)
						{
							forward_flag ++;
							back_flag = 0;
							if (forward_flag > 3 )
								chassis_register2 = 0;
							else
								chassis_register2 = 1;
						}
																																		// ���Ʊ����ʱ���� 3 ��������
						if (chassis_register2 <= 0)
						{
							back_flag ++;
							forward_flag =0;
							if(back_flag > 3)
								chassis_register2 = 1;
							else
								chassis_register2 = 0;
						}
						
					}
					
					
					if(speed_status_flag == 0)																// ��־λΪ1ʱ
				{
					if (chassis_register2 > 0)
					{	
						Chassis_Blocking_f->sign = GOFORWARD;												// ���������1�� ǰ��
					}
					
					if (chassis_register2 <= 0)															
					{	
            Chassis_Blocking_f->sign = GOBACK;														// С��1, ����
					}
		   	}	

#endif
#if (SPEED_MODE == 4)

				speed_status_count3 ++;
//				Chassis_ch2 = chassis_register1;													

        if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))			 
				{
					chassis_flag = 1; 																																		// ��ʱ���̺��˿����µ�����						
					speed_time_forward_flag = SPEED_TIME_FLAG;																						// ����������һ��ת̬�ļ�������
					speed_status_count3 = 0;
				}
				if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
				{
					chassis_flag = 2;
					speed_time_back_flag = SPEED_TIME_FLAG;
					speed_status_count3 = 0;
				}
				
			
			if(speed_status_flag == 0)																															// ��Եת��������
			{
				if (chassis_flag == 1)																															
				{
					if(speed_status_count3 > speed_time_back_flag && Chassis_Blocking_f->sign == GOBACK)		// ��ʱ������ֵ���� -> ����ʼ��������
					{ 
						Chassis_Blocking_f->sign = GOFORWARD;
						speed_status_count3 = 0;
						speed_time_back_flag += SPEED_TIME_FLAG;																					// ����ֵ�ﵽָ��ֵ -> ָ��ֵ����һ������
					}
				}
				else if(chassis_flag == 2)
				{
					if(speed_status_count3 > speed_time_forward_flag && Chassis_Blocking_f->sign == GOFORWARD)
					{
						Chassis_Blocking_f->sign = GOBACK;
						speed_status_count3 = 0;
						speed_time_forward_flag += SPEED_TIME_FLAG;
					}
					
				}
			}
				
#endif	
#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			          //������
				
				changed_HP = last_remain_HP - referee_remain_HP();										// �仯��Ѫ��
				last_remain_HP = referee_remain_HP();																	// �ϴ�Ѫ��
				speed_status_count4++;
				
				if(speed_status_count4 > 0xFFF0)				                              // ��ֹ�������															
					speed_status_count4 = 0;
				
				if(changed_HP >= 10)
			{
					if(speed_status_count4 > SPEED_TIME_FLAG)                         // ����Ƶ������
				{
						if(speed_status_flag == 0)																			// ��Եת��������
						{	
							(Chassis_Blocking_f->sign == GOBACK)? (Chassis_Blocking_f->sign = GOFORWARD) : (Chassis_Blocking_f->sign = GOBACK);			// ������
							chassis_register1 = CHASSIS_BLOCKING_SPPED;
							
							speed_status_count = 0;
							speed_status_count2 = 0; 
							#if (SPEED_MODE == 4)																																												// ��ֹƵ��ת���µ������
							speed_status_count3 = 0; 
							#endif	
							#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
							speed_status_count4 = 0;
							#endif
						}
				}	
			}
					 		
#endif				
				     
}



/**
  * @brief          ����ң��
  * @param[in]      *Chassis_Independent_f���������ṹ��
  * @retval         none
  */
uint8_t back = 0 ,forward = 0 ;

 static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f)
{
    Chassis_ch0 = Chassis_RemoteControl_f->chassis_RC->rc.ch[2];
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

