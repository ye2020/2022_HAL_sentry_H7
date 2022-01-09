/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       Task_AboveGimbal.c/h
  * @brief      上云台任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
	
#include "Task_AboveGimbal.h"	
#include "SysInit.h"


/**************** 变量 *******************/ 
gimbal_control_t Abovegimbal_control;



/**************** 函数声明 *******************/ 
static void AboveGimbal_init(gimbal_control_t *Gimbal_data_init_f);
static void Above_gimbal_controlwork(gimbal_control_t *gimbal_task_control);




/**
  * @brief         上云台主控任务
  * @param[in]      none
  * @retval         none
  */
void AboveGimbal_TASK(void const * argument)
{
	    //空闲一段时间
    vTaskDelay(AboveGimbal_TASK_INIT_TIME);	

    // 初始化
    AboveGimbal_init(&Abovegimbal_control);
  while(1)
  {
      LEDE4(1);
     // 云台主控函数 
    Above_gimbal_controlwork(&Abovegimbal_control);

			//检测周期
		vTaskDelay(AboveGimbal_CONTROL_TIME);
  }

}


/**
  * @brief         上云台初始化函数
  * @param[in]      *Gimbal_data_init_f ： 上云台控制量指针
  * @retval         none
  */
static void AboveGimbal_init(gimbal_control_t *Gimbal_data_init_f)
{
    {
        /*--------------------获取指针--------------------*/
        // 获取遥控器指针(数据)
        Gimbal_data_init_f->gimbal_RC = get_remote_control_point();
        // 获取云台电机指针
        Gimbal_data_init_f->pitch_c.pitch_motor_measure = Get_Pitch_AboveGimbal_Motor_Measure_Point();
        Gimbal_data_init_f->yaw_c.yaw_motor_measure     = Get_Yaw_AboveGimbal_Motor_Measure_Point();
        // 获取自瞄指针
        Gimbal_data_init_f->auto_c = Get_Auto_Control_Point();
        // 获取火控指针
        Gimbal_data_init_f->Fire_task_control = get_Fire_control_point();
    }

     /*--------------------检查遥控器数值是否正确--------------------*/
      RC_data_is_error();


      /*--------------------滤波初始化--------------------*/
    {
        //pitch轴低通滤波
        first_order_filter_init(&Gimbal_data_init_f->pitch_c.LowFilt_Pitch_Data, Gimbal_Pitch_Fir_Ord_Low_Fil_Param);
        //初始化P轴滑动滤波器
        Sliding_Mean_Filter_Init(&Gimbal_data_init_f->pitch_c.Slidmean_Pitch_Data);
    }

    // 控制pitch轴pid初始化
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_p_pid, ABOVE_GIMBAL_P_PITCH_P, ABOVE_GIMBAL_P_PITCH_I, ABOVE_GIMBAL_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_p_pid.maximum = 180.0f;   //180.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.minimum = -200.0f;  //-250.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.stepIn = 10.0f;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmin = 1;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmax = 8;
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_s_pid, ABOVE_GIMBAL_S_PITCH_P, ABOVE_GIMBAL_S_PITCH_I, ABOVE_GIMBAL_S_PITCH_D, 0, 0);


    // 自瞄pitch轴pid初始化
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_p_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_P, ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_I, ABOVE_GIMBAL_AUTO_INDUSTRY_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.maximum = 180.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.minimum = -250.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.stepIn = 7.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmin = 10;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmax = 30;
		pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_s_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_P, ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_I, ABOVE_GIMBAL_AUTO_INDUSTRY_S_PITCH_D, 0, 0); 

     //Yaw的pid参数初始化
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_p_pid, ABOVE_GIMBAL_P_YAW_P, ABOVE_GIMBAL_P_YAW_I, ABOVE_GIMBAL_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_s_pid, ABOVE_GIMBAL_S_YAW_P, ABOVE_GIMBAL_S_YAW_I, ABOVE_GIMBAL_S_YAW_D, 0, 0);

		//自瞄：Yaw的pid参数初始化
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_p_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_P, ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_I, ABOVE_GIMBAL_AUTO_INDUSTRY_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_s_pid, ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_P, ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_I, ABOVE_GIMBAL_AUTO_INDUSTRY_S_YAW_D, 0, 0);

    /*--------------------设置开机状态--------------------*/
  //  gimbal_task_off();

}


/**
  * @brief          上云台主要状态控制函数
  * @param[in]      none
  * @retval         none
  * @attention
  */
 static void Above_gimbal_controlwork(gimbal_control_t *gimbal_task_control)
{
        // 检查遥控器数值是否正确
        RC_data_is_error();

        //通过串口三接收并保存MiniPC发来的数据
        MiniPC_Data_Deal();  //通过串口三接收并保存MiniPC发来的数据

        //发送敌人颜色，p轴角度，射速给视觉
        MiniPC_Send_Data(1,25,(uint8_t)28);
}
VisionStatus_E  get_Enemy_status_from_above(void)
{
	return Abovegimbal_control.VisionStatus;
}
