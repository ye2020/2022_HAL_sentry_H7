/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       fire_task.c/h
  * @brief     	��̨����
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/
	
#include "Task_Gimbal.h"
#include "SysInit.h"

/**************** ���� *******************/ 

//��������̨����

gimbal_control_t gimbal_control;
#if (PITCH_PID_MODE == 1)
static float Last_ch3 = 0;
static float Now_ch3 = 0;
#endif


static uint16_t pitch_angle_init_flag = 0;		// ��־λ����ʼ����������õ�
static uint16_t pitch_angle_count = 0;				// ����ֵ����ʼ����������õ�			
/**************** �������� *******************/ 

static void Gimbal_init(gimbal_control_t *Gimbal_data_init_f);                  // ��̨���ݳ�ʼ��
static void gimbal_controlwork(gimbal_control_t *gimbal_task_control);          // ��̨��Ҫ���ƺ���
static void Gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose);     // ��̨״̬ѡ��
static void gimbal_task_off(void);
static void gimbal_pitch_init(void);


void GIMBAL_TASK(void const * argument)
{
	 vTaskDelay(GIMBAL_TASK_INIT_TIME);
    
    // ��̨���ݳ�ʼ�� 
    Gimbal_init(&gimbal_control);

	while(1)
	{
		if (pitch_angle_init_flag == 0 )
			gimbal_pitch_init();
		
		/* �������� */
		LEDE3(0);

    // ��̨��Ҫ���ƺ���
    gimbal_controlwork(&gimbal_control);

		        /* ����pitch��3508�����������������Ŀ��� ��������Ϊ- +*/
		CAN1_Gimbal_Fire(gimbal_control.Fire_task_control->GDA_output,
										 gimbal_control.Fire_task_control->GDB_output,
										 gimbal_control.pitch_c.output,
										 0); 
		
						/* ����yaw��3508������� */
    #if (gimbal_yaw_TO_chassis == 2)        
		CAN2_yaw_Setmsg(gimbal_control.yaw_c.output);               // ֱ�ӷ����
    #elif (gimbal_yaw_TO_chassis == 1)
    CAN2_Gimbal_yaw_Setmsg(gimbal_control.yaw_c.output);        // �����̣��ɵ���can1�����
    #endif
		
	
		vTaskDelay(GIMBAL_CONTROL_TIME_MS); //ϵͳ��ʱ
		
	}
	

}


/* ���̳�ʼ������ */
static void gimbal_pitch_init(void)
{ 
	#if (pitch_angle_position == 0)
		CAN1_Gimbal_Fire(0,0,-4000,0);
	vTaskDelay(2); //ϵͳ��ʱ
	
		if(gimbal_control.pitch_c .pitch_motor_measure -> speed < 100) //
		{
			pitch_angle_count ++;
			if(pitch_angle_count > 250 )
			{
				gimbal_control.pitch_c.pitch_motor_measure->actual_Position = 0;
				pitch_angle_init_flag = 1;
			}
		}
		else 
			pitch_angle_count = 0;
	#endif		
	#if (pitch_angle_position == 1)	
			pitch_angle_init_flag = 1;

	#endif
}


/**
  * @brief          ��̨���ݳ�ʼ��
  * @param[in]      none
  * @retval         none
  * @attention
  */

static void Gimbal_init(gimbal_control_t *Gimbal_data_init_f)
{
    {
      /*--------------------��ȡָ��--------------------*/
          //��ȡң����ָ��(����)
        Gimbal_data_init_f->gimbal_RC = get_remote_control_point();
          //��ȡ��̨���ָ��
        Gimbal_data_init_f->yaw_c.yaw_motor_measure = Get_Yaw_Gimbal_Motor_Measure_Point();
        Gimbal_data_init_f->pitch_c.pitch_motor_measure = Get_Pitch_Gimbal_Motor_Measure_Point();
          //��ȡ����ָ��
        Gimbal_data_init_f->auto_c = Get_Auto_Control_Point();
          // ��ȡ���ָ��
        Gimbal_data_init_f->Fire_task_control = get_Fire_control_point();
    }

       /*--------------------���ң������ֵ�Ƿ���ȷ--------------------*/
    RC_data_is_error();


      /*--------------------�˲���ʼ��--------------------*/
    {
        //pitch���ͨ�˲�
        first_order_filter_init(&Gimbal_data_init_f->pitch_c.LowFilt_Pitch_Data, Gimbal_Pitch_Fir_Ord_Low_Fil_Param);
        //��ʼ��P�Ử���˲���
        Sliding_Mean_Filter_Init(&Gimbal_data_init_f->pitch_c.Slidmean_Pitch_Data);
    }

#if (PITCH_PID_MODE == 1)	
              //Pitch������pid������ʼ��
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_p_pid, GIMBAL_UP_P_PITCH_P, GIMBAL_UP_P_PITCH_I, GIMBAL_UP_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_s_pid, GIMBAL_UP_S_PITCH_P, GIMBAL_UP_S_PITCH_I, GIMBAL_UP_S_PITCH_D, 0, 0);
        //Pitch�½���pid������ʼ��
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_p_pid, GIMBAL_DOWN_P_PITCH_P, GIMBAL_DOWN_P_PITCH_I, GIMBAL_DOWN_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_s_pid, GIMBAL_DOWN_S_PITCH_P, GIMBAL_DOWN_S_PITCH_I, GIMBAL_DOWN_S_PITCH_D, 0, 0);
				
				//���飺Pitch������pid������ʼ��
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_auto_p_pid, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_auto_s_pid, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I, 0, 0);
			
        //���飺Pitch���½�pid������ʼ��
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_auto_p_pid, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_auto_s_pid, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_D, 0, 0);



#elif (PITCH_PID_MODE == 2)	
			
        //Pitch pid������ʼ��
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_p_pid, GIMBAL_P_PITCH_P, GIMBAL_P_PITCH_I, GIMBAL_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_p_pid.maximum = 180.0f;   //180.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.minimum = -200.0f;  //-250.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.stepIn = 10.0f;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmin = 1;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmax = 8;
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_s_pid, GIMBAL_S_PITCH_P, GIMBAL_S_PITCH_I, GIMBAL_S_PITCH_D, 0, 0);
			
			//���飺Pitch��pid������ʼ��
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.maximum = 180.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.minimum = -250.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.stepIn = 7.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmin = 10;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmax = 30;
		pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_S_PITCH_D, 0, 0);
		
#endif	

        //Yaw��pid������ʼ��
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_p_pid, GIMBAL_P_YAW_P, GIMBAL_P_YAW_I, GIMBAL_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_s_pid, GIMBAL_S_YAW_P, GIMBAL_S_YAW_I, GIMBAL_S_YAW_D, 0, 0);

		//���飺Yaw��pid������ʼ��
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_YAW_P, GIMBAL_AUTO_INDUSTRY_P_YAW_I, GIMBAL_AUTO_INDUSTRY_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_YAW_P, GIMBAL_AUTO_INDUSTRY_S_YAW_I, GIMBAL_AUTO_INDUSTRY_S_YAW_D, 0, 0);
		
	/*--------------------���ÿ���״̬--------------------*/
    gimbal_task_off();

}


/**
  * @brief          ��̨��Ҫ״̬���ƺ���
  * @param[in]      none
  * @retval         none
  * @attention
  */
 static void gimbal_controlwork(gimbal_control_t *gimbal_task_control)
 {
        // ���ң������ֵ�Ƿ���ȷ
        RC_data_is_error();

        //ͨ�����������ղ�����MiniPC����������
        MiniPC_Data_Deal();  //ͨ�����������ղ�����MiniPC����������

        //���͵�����ɫ��p��Ƕȣ����ٸ��Ӿ�
        MiniPC_Send_Data(1,25,(uint8_t)20);

        //���͵��˳���״̬�����̰�
//        CAN2_Enemy_status(gimbal_task_control->VisionStatus);
 
        //��̨ģʽѡ��
        Gimbal_remote_mode_choose(gimbal_task_control);

 }


/**
  * @brief          ��̨��Ϊѡ��
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose)
{
    Gimbal_behaviour_mode_set(fir_gimbal_choose);
}



/**
  * @brief          ��̨��������
  * @param[in]      gimbal_working : ����ṹ��
  * @param[in]      gimbal_ch2     ������yaw�������
  * @param[in]      gimbal_ch3     ������pit�������
  * @retval         none
  * @attention
  */
void Gimbal_Manual_Work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3)
{

  float Gimbal_yaw_set_position = 1;        // yaw ��λ���趨

 #if( PITCH_PID_MODE == 1)
    /* P����ٶ� �ٶȵ�PID΢�� */
    gimbal_working->pitch_c.accel_down =  gimbal_working->pitch_c.pitch_down_s_pid.Derror[0] * 100.0f;
    gimbal_working->pitch_c.accel_up = gimbal_working->pitch_c.pitch_up_s_pid.Derror[0] * 100.0f;

    Now_ch3 = gimbal_ch3;
    Last_ch3 = Now_ch3;
#endif

       /* P���˲� */
    gimbal_working->pitch_c.output = Sliding_Mean_Filter(&gimbal_working->pitch_c.Slidmean_Pitch_Data, gimbal_working->pitch_c.output, 55); //��ֵ�����˲������ͺ�
    gimbal_working->pitch_c.output = first_order_filter(&gimbal_working->pitch_c.LowFilt_Pitch_Data, gimbal_working->pitch_c.output);       //һ�׵�ͨ�˲�


#if (PITCH_PID_MODE == 1)

        if (Now_ch3 - Last_ch3 > 0.0f)
        {
            /* Pitchλ�ÿ��� */
            gimbal_working->pitch_c.output = 
            Motor_Position_Speed_Control(&gimbal_working->pitch_c.pitch_up_s_pid,
                                         &gimbal_working->pitch_c.pitch_up_p_pid,
                                          gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,             /*��ʵλ��*/
                                          gimbal_working->pitch_c.pitch_motor_measure->speed,                   /*��ʵ�ٶ�*///IMU_t.Gyro_X
                                          (gimbal_ch3),                                                         /*�趨λ��*///�����ر�����:1024  3508:8192
                                          PITCH_OUTPUT_LIMIT);                                         				 /*�������*/
        }
        else
        {
            /* Pitchλ�ÿ��� */
            gimbal_working->pitch_c.output = Motor_Position_Speed_Control(&gimbal_working->pitch_c.pitch_down_s_pid,
                                             &gimbal_working->pitch_c.pitch_down_p_pid,
                                             gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,               /*��ʵλ��*/
                                             gimbal_working->pitch_c.pitch_motor_measure->speed,                     /*��ʵ�ٶ�*///IMU_t.Gyro_X
                                             (gimbal_ch3),  /*�趨λ��*/
                                             PITCH_OUTPUT_LIMIT);                                             		   /*�������*/
        }
#elif (PITCH_PID_MODE == 2) 

        /* ����pid */
        gimbal_working->pitch_c.output =
                motor_position_Stepping   (&gimbal_working->pitch_c.pitch_s_pid,
                                           &gimbal_working->pitch_c.pitch_p_pid,
                                            gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,     // ��ʵλ��
                                            gimbal_working->pitch_c.pitch_motor_measure->speed,           // ��ʵ�ٶ�
                                            (gimbal_ch3)	,                                               // �趨λ��
                                            PITCH_OUTPUT_LIMIT );                                         // �������                                                   
#endif
        Gimbal_yaw_set_position = loop_fp32_constrain((gimbal_ch2 - gimbal_working->yaw_c.yaw_motor_measure->yaw_angle) , -180.0f, 180.0f);    //���������̨�趨λ��ѭ���޷�

      /* Yawλ�ÿ��� ���ڱ��ķ������� yaw�������Ҹ� pitch���ϸ����� */
        gimbal_working->yaw_c.output  = 
              Motor_Position_Speed_Control(&gimbal_working->yaw_c.yaw_s_pid,
                                           &gimbal_working->yaw_c.yaw_p_pid,
                                           0,                                                    // ��ʵλ��
                                           gimbal_working->yaw_c.yaw_motor_measure->speed,       // ��ʵ�ٶ�
                                           Gimbal_yaw_set_position,                              // �趨λ��
                                           YAW_OUTPUT_LIMIT);                                    // �������


}



/**
  * @brief          ����ģʽ
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Gimbal_Automatic_Work(gimbal_control_t *gimbal_automatic_work_f)
{
    /*yaw��������ջ�����*/
    gimbal_automatic_work_f->yaw_c.output = 
               Motor_Position_Speed_Control(&gimbal_automatic_work_f->yaw_c.yaw_auto_s_pid,
                                            &gimbal_automatic_work_f->yaw_c.yaw_auto_p_pid,
                                            0,                                                          // ��ʵλ��
                                            gimbal_automatic_work_f->yaw_c.yaw_motor_measure->speed,    // ��ʵ�ٶ�
                                            (gimbal_automatic_work_f->auto_c->yaw_control_data),        // �趨λ��
                                            YAW_OUTPUT_LIMIT);                                          // �������

  #if (PITCH_PID_MODE == 1)

    Now_ch3 = gimbal_automatic_work_f->pitch_c.Auto_record_location;
    Last_ch3 = Now_ch3;
    if (Now_ch3 - Last_ch3 > 0.0f)//max δ��		min -10
    {
				gimbal_automatic_work_f->pitch_c.output = Motor_Position_Speed_Control(&gimbal_automatic_work_f->pitch_c.pitch_up_auto_p_pid,
																									&gimbal_automatic_work_f->pitch_c.pitch_up_auto_p_pid,
																									-gimbal_automatic_work_f->pitch_c.Auto_record_location ,																																		/*��ʵλ��*/
																									gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
																									(-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data),							/*�趨λ��*/
																									PITCH_OUTPUT_LIMIT);
		}
		else
		{
				gimbal_automatic_work_f->pitch_c.output = Motor_Position_Speed_Control(&gimbal_automatic_work_f->pitch_c.pitch_down_auto_p_pid,
																									&gimbal_automatic_work_f->pitch_c.pitch_down_auto_p_pid,
																									-gimbal_automatic_work_f->pitch_c.Auto_record_location ,																																		/*��ʵλ��*///
																									gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
																									(-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data),							/*�趨λ��*/
																									PITCH_OUTPUT_LIMIT);
		}
  #elif (PITCH_PID_MODE == 2)
  
  gimbal_automatic_work_f->pitch_c.output = 
                    motor_position_Stepping(&gimbal_automatic_work_f->pitch_c.pitch_auto_s_pid,
                                            &gimbal_automatic_work_f->pitch_c.pitch_auto_p_pid, 
                                            -gimbal_automatic_work_f->pitch_c.Auto_record_location ,  
                                            gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
                                            (-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data), 
                                            PITCH_OUTPUT_LIMIT);
  #endif

  
}

/**
  * @brief          ��̨ȫ���ر�
  * @param[in]      none
  * @retval         none
  * @attention      0: ״̬���л�
  *                 1: ���ν���״̬
  *                 2: �����ʼ����־λ
  */
static void gimbal_task_off(void)
{
      //������
    gimbal_control.pitch_c.output = 0;
    gimbal_control.yaw_c.output = 0;

    gimbal_control.VisionStatus = Enemy_Disappear;        
    // ���������
//    gimbal_control.auto_c->auto_pitch_angle = 0.0f;
//    gimbal_control.auto_c->auto_yaw_angle = 0.0f;
//    gimbal_control.auto_c->pitch_control_data = 0.0f;
//    gimbal_control.auto_c->yaw_control_data = 0.0f;
//    gimbal_control.pitch_c.Auto_record_location = 0.0f;

      gimbal_control.gimbal_behaviour = GIMBAL_STOP;
      Gimbal_Stop(&gimbal_control);
}

/**
  * @brief          ������̨ģʽ
  * @param[in]      none
  * @retval        
  * @attention      
  */
 gimbal_behaviour_e Return_gimbal_mode(void)
 {
      return gimbal_control.gimbal_behaviour;
 }
