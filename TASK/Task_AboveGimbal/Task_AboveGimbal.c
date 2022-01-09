/**
  *****************************��ݸ��ѧԺACEʵ���� *****************************
  * @file       Task_AboveGimbal.c/h
  * @brief      ����̨����
  * @note       �ϲ��汾
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************��ݸ��ѧԺACEʵ���� *****************************
	*/
	
#include "Task_AboveGimbal.h"	
#include "SysInit.h"


/**************** ���� *******************/ 
gimbal_control_t Abovegimbal_control;



/**************** �������� *******************/ 
static void AboveGimbal_init(gimbal_control_t *Gimbal_data_init_f);
static void Above_gimbal_controlwork(gimbal_control_t *gimbal_task_control);




/**
  * @brief         ����̨��������
  * @param[in]      none
  * @retval         none
  */
void AboveGimbal_TASK(void const * argument)
{
	    //����һ��ʱ��
    vTaskDelay(AboveGimbal_TASK_INIT_TIME);	

    // ��ʼ��
    AboveGimbal_init(&Abovegimbal_control);
  while(1)
  {
      LEDE4(1);
     // ��̨���غ��� 
    Above_gimbal_controlwork(&Abovegimbal_control);

			//�������
		vTaskDelay(AboveGimbal_CONTROL_TIME);
  }

}


/**
  * @brief         ����̨��ʼ������
  * @param[in]      *Gimbal_data_init_f �� ����̨������ָ��
  * @retval         none
  */
static void AboveGimbal_init(gimbal_control_t *Gimbal_data_init_f)
{
    {
        /*--------------------��ȡָ��--------------------*/
        // ��ȡң����ָ��(����)
        Gimbal_data_init_f->gimbal_RC = get_remote_control_point();
        // ��ȡ��̨���ָ��
        Gimbal_data_init_f->pitch_c.pitch_motor_measure = Get_Pitch_AboveGimbal_Motor_Measure_Point();
        Gimbal_data_init_f->yaw_c.yaw_motor_measure     = Get_Yaw_AboveGimbal_Motor_Measure_Point();
        // ��ȡ����ָ��
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

    // ����pitch��pid��ʼ��
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_p_pid, ABOVE_GIMBAL_P_PITCH_P, ABOVE_GIMBAL_P_PITCH_I, ABOVE_GIMBAL_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_p_pid.maximum = 180.0f;   //180.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.minimum = -200.0f;  //-250.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.stepIn = 10.0f;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmin = 1;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmax = 8;
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_s_pid, ABOVE_GIMBAL_S_PITCH_P, ABOVE_GIMBAL_S_PITCH_I, ABOVE_GIMBAL_S_PITCH_D, 0, 0);


    // ����pitch��pid��ʼ��
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_p_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_P, ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_I, ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.maximum = 180.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.minimum = -250.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.stepIn = 7.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmin = 10;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmax = 30;
		pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_s_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_P, ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_I, ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_D, 0, 0); 

     //Yaw��pid������ʼ��
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_p_pid, ABOVE_GIMBAL_P_YAW_P, ABOVE_GIMBAL_P_YAW_I, ABOVE_GIMBAL_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_s_pid, ABOVE_GIMBAL_S_YAW_P, ABOVE_GIMBAL_S_YAW_I, ABOVE_GIMBAL_S_YAW_D, 0, 0);

		//���飺Yaw��pid������ʼ��
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_p_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_P, ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_I, ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_s_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_P, ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_I, ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_D, 0, 0);

    /*--------------------���ÿ���״̬--------------------*/
  //  gimbal_task_off();

}


/**
  * @brief          ����̨��Ҫ״̬���ƺ���
  * @param[in]      none
  * @retval         none
  * @attention
  */
 static void Above_gimbal_controlwork(gimbal_control_t *gimbal_task_control)
{
        // ���ң������ֵ�Ƿ���ȷ
        RC_data_is_error();

        //ͨ�����������ղ�����MiniPC����������
        MiniPC_Data_Deal();  //ͨ�����������ղ�����MiniPC����������

        //���͵�����ɫ��p��Ƕȣ����ٸ��Ӿ�
        MiniPC_Send_Data(1,25,(uint8_t)28);
}
VisionStatus_E  get_Enemy_status_from_above(void)
{
	return Abovegimbal_control.VisionStatus;
}
