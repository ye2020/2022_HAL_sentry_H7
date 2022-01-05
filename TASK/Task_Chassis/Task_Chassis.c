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
	
#include "Task_Chassis.h"
#include "SysInit.h"
#include "rng.h"
/**************** �������� *******************/ 

static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f);       // ����ң��ģʽѡ��
static void chassis_controlwork(chassis_control_t *chassis_control_f);                  // ������Ҫ������ƺ���
static void chassis_init(chassis_control_t *chassis_move_init_f);                       // ���̳�ʼ��
static void chassis_data_update(chassis_control_t *chassis_data_update_f);              // �������ݸ���
static void chassis_pid_calc(chassis_control_t *chassis_pid_f);                         // pid ����       
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2);      // ���̼��ٶ�����б�º���
static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f);											// ���̷������ݵ���̨


/********************************************/
//���̿������� static
 chassis_control_t chassis_control;


void Chassis_Task(void const * argument)
	{
	    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);	
		
    /*���̳�ʼ��*/
    chassis_init(&chassis_control);

		while(1)
		{
			
      /* ���̿��� */
      chassis_controlwork(&chassis_control);


   #if chassis_using
			LEDE2(0);
    // ���̷��͵���ֵ
      Chassis_CAN_Send_Msg(chassis_control.chassis_motor[0].output,
													 chassis_control.chassis_motor[1].output,
													 chassis_control.chassis_motor[2].output,
													 chassis_control.chassis_motor[3].output  );	
  #endif

  #if (gimbal_yaw_TO_chassis == 1)
    
      CAN1_Chassis_yaw_Setmsg(Get_Yaw_Gimbal_Motor_Output());

  #endif
			//�������
		vTaskDelay(CHASSIS_CONTROL_TIME);
		
		}                                                                                                                            
		
	}
	

	
/**
  * @brief          ����ң��ģʽѡ��
  * @param[in]      *chassis_mode_choose_f���������ṹ��
  * @retval         none
  */
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f)
{
    //�ж���ʲôģʽ
    chassis_behaviour_mode_set(chassis_mode_choose_f);
}



/**
  * @brief          ������Ҫ������ƺ���
  * @param[in]      *chassis_control_f���������ṹ��
  * @retval         none
  */
static void chassis_controlwork(chassis_control_t *chassis_control_f)
{
    // ���ң������ֵ�Ƿ���ȷ
    RC_data_is_error();

    // �������ݸ���
    chassis_data_update(chassis_control_f);

    // ң����ģʽ״̬����
    chassis_remote_mode_choose(chassis_control_f);

    //���̿���PID����
    chassis_pid_calc(chassis_control_f);

    // ���̷������ݸ���̨
		Chassis_to_Gimbal(chassis_control_f);

}


/**
  * @brief          �������ݳ�ʼ��
  * @param[in]      *chassis_move_init_f���������ṹ��
  * @retval         none
  */
static void chassis_init(chassis_control_t *chassis_move_init_f)
{
  static uint8_t i;         // ѡ����

      /*--------------------��ȡָ��--------------------*/
    {
            //����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
        chassis_move_init_f->chassis_RC = get_remote_control_point();
            //��ȡ����ָ��
                for (i = 0; i < 4; i++)
        {
            chassis_move_init_f->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        }

           
      #if (gimbal_yaw_TO_chassis == 1)
      // yaw������ ���������͸����̼��㣩
      chassis_move_init_f->yaw_motor_measure = Chassis_Get_Yaw_Gimbal_Motor_Measure_Point();
      #endif 

    }
    
    /*--------------------���ң������ֵ�Ƿ���ȷ--------------------*/
    RC_data_is_error();
		
		
    /*--------------------��ʼ����ͨ�˲�--------------------*/
    {
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);
    }

     /*--------------------��ʼ������pid--------------------*/
    // �����õ��ľ�motor[0], ���ﻹ�ǳ�ʼ���ĸ����������������ú����ݸ��»��ǻ��ĸ����һ�𣬷�ֹ����֮����̵���������ӡ���ps:����������   _(:�١���)_   ��
    {
        //�����ƶ�pid
        pid_init(&chassis_move_init_f->chassis_speed_pid[0], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[1], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[2], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[3], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);

        //
        pid_init(&chassis_move_init_f->chassis_location_pid, CHASSIS_LOCATION_PID_P, CHASSIS_LOCATION_PID_I, CHASSIS_LOCATION_PID_D, 0, 0);
    }

    //���̿���״̬Ϊֹͣ (ע������ڻ�ȡָ���Ժ�)
    Chassis_Task_OFF(2);;

    // ����ʱ��ʼ��ǰ������
    chassis_move_init_f->sign = GOFORWARD;

    // ���ν����������
    chassis_data_update(chassis_move_init_f);

    
}


/**
  * @brief          ���̷������ݵ���̨��yaw������������ֵ��yaw�����ٶ�ֵ��ң��ֵ��
  * @param[in]      *chassis_setmsg_f���������ṹ��
  * @retval         none
  */
static int send_sign = 0; //˳����ң��ֵ

static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f)
{
	if ((chassis_setmsg_f->chassis_mode != CHASSIS_STANDBY) || (chassis_setmsg_f->chassis_mode != CHASSIS_STOP)) //����ˢ������
	{
      if (send_sign == 0)
    {
      CAN2_Chassis_RC_SetMsg(chassis_setmsg_f->chassis_RC); //����Ϊң����ģʽ
      send_sign = 1;
    }
      
    if (send_sign == 1)
    {
      #if (gimbal_yaw_TO_chassis == 1)
      CAN2_Chassis_YAW_SetMsg(chassis_setmsg_f->yaw_motor_measure);
      #endif
			send_sign = 0;

    }
  }
}


/**
  * @brief          ���̼��ٶ�����б�º���
  * @param[in]      *ch0��
  *                 *ch1��
  *                 *ch2��
  * @retval         б�º���������Ŀ�ľ����������źű�ø���ƽ��������ϵͳ�������Ӷ��Ż�ϵͳ��ʱ����Ӧ��
  */
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2)
{
    static int16_t last_ch[3] = {0, 0, 0};
    int16_t temp[3];

    temp[0] = *ch0 - last_ch[0];
    temp[1] = *ch1 - last_ch[1];
    temp[2] = *ch2 - last_ch[2];

    if (chassis_control.chassis_mode == CHASSIS_AUTO) //�Զ�ģʽ
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;

        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;
    }

    if (chassis_control.chassis_mode == CHASSIS_REMOTECONTROL) //ң��ģʽ
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;

        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;
    }

    last_ch[0] = *ch0;
    last_ch[1] = *ch1;
    last_ch[2] = *ch2;
}

/**
  * @brief          �������ݸ���
  * @param[in]      *chassis_data_update_f���������ṹ��
  * @retval         none
  * @attention
  */
static void chassis_data_update(chassis_control_t *chassis_data_update_f)
{
    uint8_t i;

    //����ٶȸ���
    for (i = 0; i < 4; i++)
    {
        chassis_data_update_f->chassis_motor[i].speed = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->speed;
        chassis_data_update_f->chassis_motor[i].position = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->position;
        chassis_data_update_f->chassis_motor[i].accel = chassis_data_update_f->chassis_speed_pid[i].Derror[0] * 500.0f;
    }
}


/**
  * @brief          ���̿���PID����
  * @param[in]      *chassis_pid_f���������ṹ��
  * @retval         none
  */
static void chassis_pid_calc(chassis_control_t *chassis_pid_f)
{
  // �ٶȻ�pid ������������ٶȻ�pid�ṹ��spid��Ŀ���ٶ�ֵsetSpeed��ʵ���ٶ�ֵactualSpeed �������޷� current_limit��

	 chassis_pid_f->chassis_motor[0].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[0]), chassis_pid_f->chassis_motor[0].speed_set, chassis_pid_f->chassis_motor[0].speed, M3508_MAX_OUTPUT_CURRENT); //M3508_MAX_OUTPUT_CURRENT
   chassis_pid_f->chassis_motor[1].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[1]), chassis_pid_f->chassis_motor[1].speed_set, chassis_pid_f->chassis_motor[1].speed, M3508_MAX_OUTPUT_CURRENT); //���16000���� MAX_MOTOR_CAN_OUTPUT
   chassis_pid_f->chassis_motor[2].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[2]), chassis_pid_f->chassis_motor[2].speed_set, chassis_pid_f->chassis_motor[2].speed, M3508_MAX_OUTPUT_CURRENT);
   chassis_pid_f->chassis_motor[3].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[3]), chassis_pid_f->chassis_motor[3].speed_set, chassis_pid_f->chassis_motor[3].speed, M3508_MAX_OUTPUT_CURRENT);



}


/**
  * @brief          ���̿���������
  * @param[in]      *chassis_set_f���������ṹ��
  *                 ch0��������ch0����ֵ
  *                 ch1��������ch1����ֵ
  *                 ch2��������ch2����ֵ
  * @retval         none
  */
 void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2)
{
      /*������б�º�������*/
    Chassis_accelerated_Control(&ch0, &ch1, &ch2);

        //һ�׵�ͨ�˲�����
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vx), -ch0);
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vy), ch1);

        /*����ң��ģʽ*/
    if (chassis_set_f->chassis_mode == CHASSIS_REMOTECONTROL)
    {			
        chassis_set_f->speed_x_set =5.0f * ch0;
        chassis_set_f->chassis_motor[0].speed_set = chassis_set_f->speed_x_set;
    }
        /* ����ģʽ */
    if (chassis_set_f->chassis_mode == CHASSIS_STANDBY)
    {

        chassis_set_f->speed_x_set = 0.0f * ch0;
			
        chassis_set_f->chassis_motor[0].speed_set = chassis_set_f->speed_x_set;
    }
        /*�����Զ�ģʽ*/
   if  (chassis_set_f->chassis_mode == CHASSIS_AUTO || chassis_set_f->chassis_mode == CHASSIS_BLOCKING) 
   {
        chassis_set_f ->speed_x_set = ch2;

         if (chassis_set_f->sign == GOFORWARD)
         {
            chassis_set_f ->chassis_motor[0].speed_set = chassis_set_f -> speed_x_set;
         }

         if (chassis_set_f->sign == GOBACK)
         {
            chassis_set_f ->chassis_motor[0].speed_set = -chassis_set_f -> speed_x_set;
         }
   }  


}

/**
  * @brief          ����ȫ���ر�
  * @param[in]      options:  0:������̿�����  1:��ֹ����   2:��ֹ���̼�yaw��Ͳ�����
  * @retval         none
  * @attention
  */
void Chassis_Task_OFF(uint8_t options)
{
    //������̿�����
    chassis_control.speed_x_set = 0;
    chassis_control.speed_y_set = 0;
    chassis_control.chassis_motor[0].speed_set = 0;
    chassis_control.chassis_motor[1].speed_set = 0;
    chassis_control.chassis_motor[2].speed_set = 0;
    chassis_control.chassis_motor[3].speed_set = 0;
    chassis_control.chassis_motor[0].output = 0;
    chassis_control.chassis_motor[1].output = 0;
    chassis_control.chassis_motor[2].output = 0;
    chassis_control.chassis_motor[3].output = 0;

		chassis_control.chassis_mode = CHASSIS_STOP;

    if (options)
    {
        Chassis_CAN_Send_Msg(0, 0,0,0);

        if (options == 2)
        {
//            CAN1_Chassis_Gimbal_Fire(0, 0, 0);
        }
    }
}

