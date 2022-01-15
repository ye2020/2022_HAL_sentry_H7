/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       fire_task.c/h
  * @brief     	云台任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
	
#include "Task_Gimbal.h"
#include "SysInit.h"

/**************** 变量 *******************/ 

//申明主云台变量

gimbal_control_t gimbal_control;
#if (PITCH_PID_MODE == 1)
static float Last_ch3 = 0;
static float Now_ch3 = 0;
#endif


static uint16_t pitch_angle_init_flag = 0;		// 标志位，初始化电机码盘用的
static uint16_t pitch_angle_count = 0;				// 计数值，初始化电机码盘用的			
/**************** 函数声明 *******************/ 

static void Gimbal_init(gimbal_control_t *Gimbal_data_init_f);                  // 云台数据初始化
static void gimbal_controlwork(gimbal_control_t *gimbal_task_control);          // 云台主要控制函数
static void Gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose);     // 云台状态选择
static void gimbal_task_off(void);
static void gimbal_pitch_init(void);


void GIMBAL_TASK(void const * argument)
{
	 vTaskDelay(GIMBAL_TASK_INIT_TIME);
    
    // 云台数据初始化 
    Gimbal_init(&gimbal_control);

	while(1)
	{
		if (pitch_angle_init_flag == 0 )
			gimbal_pitch_init();
		
		/* 心跳任务 */
		LEDE3(0);

    // 云台主要控制函数
    gimbal_controlwork(&gimbal_control);

		        /* 进行pitch轴3508电机和两个拨弹电机的控制 两个拨弹为- +*/
		CAN1_Gimbal_Fire(gimbal_control.Fire_task_control->GDA_output,
										 gimbal_control.Fire_task_control->GDB_output,
										 gimbal_control.pitch_c.output,
										 0); 
		
						/* 进行yaw轴3508电机控制 */
    #if (gimbal_yaw_TO_chassis == 2)        
		CAN2_yaw_Setmsg(gimbal_control.yaw_c.output);               // 直接发电调
    #elif (gimbal_yaw_TO_chassis == 1)
    CAN2_Gimbal_yaw_Setmsg(gimbal_control.yaw_c.output);        // 发底盘，由底盘can1发电调
    #endif
		
	
		vTaskDelay(GIMBAL_CONTROL_TIME_MS); //系统延时
		
	}
	

}


/* 码盘初始化函数 */
static void gimbal_pitch_init(void)
{ 
	#if (pitch_angle_position == 0)
		CAN1_Gimbal_Fire(0,0,-4000,0);
	vTaskDelay(2); //系统延时
	
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
  * @brief          云台数据初始化
  * @param[in]      none
  * @retval         none
  * @attention
  */

static void Gimbal_init(gimbal_control_t *Gimbal_data_init_f)
{
    {
      /*--------------------获取指针--------------------*/
          //获取遥控器指针(数据)
        Gimbal_data_init_f->gimbal_RC = get_remote_control_point();
          //获取云台电机指针
        Gimbal_data_init_f->yaw_c.yaw_motor_measure = Get_Yaw_Gimbal_Motor_Measure_Point();
        Gimbal_data_init_f->pitch_c.pitch_motor_measure = Get_Pitch_Gimbal_Motor_Measure_Point();
          //获取自瞄指针
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

#if (PITCH_PID_MODE == 1)	
              //Pitch上升的pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_p_pid, GIMBAL_UP_P_PITCH_P, GIMBAL_UP_P_PITCH_I, GIMBAL_UP_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_s_pid, GIMBAL_UP_S_PITCH_P, GIMBAL_UP_S_PITCH_I, GIMBAL_UP_S_PITCH_D, 0, 0);
        //Pitch下降的pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_p_pid, GIMBAL_DOWN_P_PITCH_P, GIMBAL_DOWN_P_PITCH_I, GIMBAL_DOWN_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_s_pid, GIMBAL_DOWN_S_PITCH_P, GIMBAL_DOWN_S_PITCH_I, GIMBAL_DOWN_S_PITCH_D, 0, 0);
				
				//自瞄：Pitch的上升pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_auto_p_pid, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_UP_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_up_auto_s_pid, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_UP_S_PITCH_I, 0, 0);
			
        //自瞄：Pitch的下降pid参数初始化
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_auto_p_pid, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_DOWN_P_PITCH_D, 0, 0);
        pid_init(&Gimbal_data_init_f->pitch_c.pitch_down_auto_s_pid, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_DOWN_S_PITCH_D, 0, 0);



#elif (PITCH_PID_MODE == 2)	
			
        //Pitch pid参数初始化
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_p_pid, GIMBAL_P_PITCH_P, GIMBAL_P_PITCH_I, GIMBAL_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_p_pid.maximum = 180.0f;   //180.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.minimum = -200.0f;  //-250.0f
		Gimbal_data_init_f->pitch_c.pitch_p_pid.stepIn = 10.0f;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmin = 1;
		Gimbal_data_init_f->pitch_c.pitch_p_pid.errorabsmax = 8;
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_s_pid, GIMBAL_S_PITCH_P, GIMBAL_S_PITCH_I, GIMBAL_S_PITCH_D, 0, 0);
			
			//自瞄：Pitch的pid参数初始化
    pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_PITCH_P, GIMBAL_AUTO_INDUSTRY_P_PITCH_I, GIMBAL_AUTO_INDUSTRY_P_PITCH_D, 2000, 0);
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.maximum = 180.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.minimum = -250.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.stepIn = 7.0f;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmin = 10;
		Gimbal_data_init_f->pitch_c.pitch_auto_p_pid.errorabsmax = 30;
		pid_init(&Gimbal_data_init_f->pitch_c.pitch_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_PITCH_P, GIMBAL_AUTO_INDUSTRY_S_PITCH_I, GIMBAL_AUTO_INDUSTRY_S_PITCH_D, 0, 0);
		
#endif	

        //Yaw的pid参数初始化
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_p_pid, GIMBAL_P_YAW_P, GIMBAL_P_YAW_I, GIMBAL_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_s_pid, GIMBAL_S_YAW_P, GIMBAL_S_YAW_I, GIMBAL_S_YAW_D, 0, 0);

		//自瞄：Yaw的pid参数初始化
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_p_pid, GIMBAL_AUTO_INDUSTRY_P_YAW_P, GIMBAL_AUTO_INDUSTRY_P_YAW_I, GIMBAL_AUTO_INDUSTRY_P_YAW_D, 0, 0);
		pid_init(&Gimbal_data_init_f->yaw_c.yaw_auto_s_pid, GIMBAL_AUTO_INDUSTRY_S_YAW_P, GIMBAL_AUTO_INDUSTRY_S_YAW_I, GIMBAL_AUTO_INDUSTRY_S_YAW_D, 0, 0);
		
	/*--------------------设置开机状态--------------------*/
    gimbal_task_off();

}


/**
  * @brief          云台主要状态控制函数
  * @param[in]      none
  * @retval         none
  * @attention
  */
 static void gimbal_controlwork(gimbal_control_t *gimbal_task_control)
 {
        // 检查遥控器数值是否正确
        RC_data_is_error();

        //通过串口三接收并保存MiniPC发来的数据
        MiniPC_Data_Deal();  //通过串口三接收并保存MiniPC发来的数据

        //发送敌人颜色，p轴角度，射速给视觉
        MiniPC_Send_Data(1,25,(uint8_t)20);

        //发送敌人出现状态给底盘板
//        CAN2_Enemy_status(gimbal_task_control->VisionStatus);
 
        //云台模式选择
        Gimbal_remote_mode_choose(gimbal_task_control);

 }


/**
  * @brief          云台行为选择
  * @param[in]      none
  * @retval         none
  * @attention
  */
static void Gimbal_remote_mode_choose(gimbal_control_t *fir_gimbal_choose)
{
    Gimbal_behaviour_mode_set(fir_gimbal_choose);
}



/**
  * @brief          云台正常控制
  * @param[in]      gimbal_working : 传入结构体
  * @param[in]      gimbal_ch2     ：传入yaw轴控制量
  * @param[in]      gimbal_ch3     ：传入pit轴控制量
  * @retval         none
  * @attention
  */
void Gimbal_Manual_Work(gimbal_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3)
{

  float Gimbal_yaw_set_position = 1;        // yaw 轴位置设定

 #if( PITCH_PID_MODE == 1)
    /* P轴加速度 速度的PID微分 */
    gimbal_working->pitch_c.accel_down =  gimbal_working->pitch_c.pitch_down_s_pid.Derror[0] * 100.0f;
    gimbal_working->pitch_c.accel_up = gimbal_working->pitch_c.pitch_up_s_pid.Derror[0] * 100.0f;

    Now_ch3 = gimbal_ch3;
    Last_ch3 = Now_ch3;
#endif

       /* P轴滤波 */
    gimbal_working->pitch_c.output = Sliding_Mean_Filter(&gimbal_working->pitch_c.Slidmean_Pitch_Data, gimbal_working->pitch_c.output, 55); //均值滑窗滤波（有滞后）
    gimbal_working->pitch_c.output = first_order_filter(&gimbal_working->pitch_c.LowFilt_Pitch_Data, gimbal_working->pitch_c.output);       //一阶低通滤波


#if (PITCH_PID_MODE == 1)

        if (Now_ch3 - Last_ch3 > 0.0f)
        {
            /* Pitch位置控制 */
            gimbal_working->pitch_c.output = 
            Motor_Position_Speed_Control(&gimbal_working->pitch_c.pitch_up_s_pid,
                                         &gimbal_working->pitch_c.pitch_up_p_pid,
                                          gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,             /*真实位置*/
                                          gimbal_working->pitch_c.pitch_motor_measure->speed,                   /*真实速度*///IMU_t.Gyro_X
                                          (gimbal_ch3),                                                         /*设定位置*///布瑞特编码器:1024  3508:8192
                                          PITCH_OUTPUT_LIMIT);                                         				 /*输出限制*/
        }
        else
        {
            /* Pitch位置控制 */
            gimbal_working->pitch_c.output = Motor_Position_Speed_Control(&gimbal_working->pitch_c.pitch_down_s_pid,
                                             &gimbal_working->pitch_c.pitch_down_p_pid,
                                             gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,               /*真实位置*/
                                             gimbal_working->pitch_c.pitch_motor_measure->speed,                     /*真实速度*///IMU_t.Gyro_X
                                             (gimbal_ch3),  /*设定位置*/
                                             PITCH_OUTPUT_LIMIT);                                             		   /*输出限制*/
        }
#elif (PITCH_PID_MODE == 2) 

        /* 步进pid */
        gimbal_working->pitch_c.output =
                motor_position_Stepping   (&gimbal_working->pitch_c.pitch_s_pid,
                                           &gimbal_working->pitch_c.pitch_p_pid,
                                            gimbal_working->pitch_c.pitch_motor_measure->pitch_angle,     // 真实位置
                                            gimbal_working->pitch_c.pitch_motor_measure->speed,           // 真实速度
                                            (gimbal_ch3)	,                                               // 设定位置
                                            PITCH_OUTPUT_LIMIT );                                         // 输出限制                                                   
#endif
        Gimbal_yaw_set_position = loop_fp32_constrain((gimbal_ch2 - gimbal_working->yaw_c.yaw_motor_measure->yaw_angle) , -180.0f, 180.0f);    //修正后的云台设定位置循环限幅

      /* Yaw位置控制 以哨兵的方向来讲 yaw轴左正右负 pitch轴上负下正 */
        gimbal_working->yaw_c.output  = 
              Motor_Position_Speed_Control(&gimbal_working->yaw_c.yaw_s_pid,
                                           &gimbal_working->yaw_c.yaw_p_pid,
                                           0,                                                    // 真实位置
                                           gimbal_working->yaw_c.yaw_motor_measure->speed,       // 真实速度
                                           Gimbal_yaw_set_position,                              // 设定位置
                                           YAW_OUTPUT_LIMIT);                                    // 输出限制


}



/**
  * @brief          自瞄模式
  * @param[in]      none
  * @retval         none
  * @attention
  */
void Gimbal_Automatic_Work(gimbal_control_t *gimbal_automatic_work_f)
{
    /*yaw轴输出量闭环处理*/
    gimbal_automatic_work_f->yaw_c.output = 
               Motor_Position_Speed_Control(&gimbal_automatic_work_f->yaw_c.yaw_auto_s_pid,
                                            &gimbal_automatic_work_f->yaw_c.yaw_auto_p_pid,
                                            0,                                                          // 真实位置
                                            gimbal_automatic_work_f->yaw_c.yaw_motor_measure->speed,    // 真实速度
                                            (gimbal_automatic_work_f->auto_c->yaw_control_data),        // 设定位置
                                            YAW_OUTPUT_LIMIT);                                          // 输出限制

  #if (PITCH_PID_MODE == 1)

    Now_ch3 = gimbal_automatic_work_f->pitch_c.Auto_record_location;
    Last_ch3 = Now_ch3;
    if (Now_ch3 - Last_ch3 > 0.0f)//max 未测		min -10
    {
				gimbal_automatic_work_f->pitch_c.output = Motor_Position_Speed_Control(&gimbal_automatic_work_f->pitch_c.pitch_up_auto_p_pid,
																									&gimbal_automatic_work_f->pitch_c.pitch_up_auto_p_pid,
																									-gimbal_automatic_work_f->pitch_c.Auto_record_location ,																																		/*真实位置*/
																									gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
																									(-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data),							/*设定位置*/
																									PITCH_OUTPUT_LIMIT);
		}
		else
		{
				gimbal_automatic_work_f->pitch_c.output = Motor_Position_Speed_Control(&gimbal_automatic_work_f->pitch_c.pitch_down_auto_p_pid,
																									&gimbal_automatic_work_f->pitch_c.pitch_down_auto_p_pid,
																									-gimbal_automatic_work_f->pitch_c.Auto_record_location ,																																		/*真实位置*///
																									gimbal_automatic_work_f->pitch_c.pitch_motor_measure->speed,
																									(-gimbal_automatic_work_f->pitch_c.Auto_record_location - gimbal_automatic_work_f->auto_c->pitch_control_data),							/*设定位置*/
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
  * @brief          云台全部关闭
  * @param[in]      none
  * @retval         none
  * @attention      0: 状态接切换
  *                 1: 初次进入状态
  *                 2: 清除初始化标志位
  */
static void gimbal_task_off(void)
{
      //控制量
    gimbal_control.pitch_c.output = 0;
    gimbal_control.yaw_c.output = 0;

    gimbal_control.VisionStatus = Enemy_Disappear;        
    // 自瞄控制量
//    gimbal_control.auto_c->auto_pitch_angle = 0.0f;
//    gimbal_control.auto_c->auto_yaw_angle = 0.0f;
//    gimbal_control.auto_c->pitch_control_data = 0.0f;
//    gimbal_control.auto_c->yaw_control_data = 0.0f;
//    gimbal_control.pitch_c.Auto_record_location = 0.0f;

      gimbal_control.gimbal_behaviour = GIMBAL_STOP;
      Gimbal_Stop(&gimbal_control);
}

/**
  * @brief          返回云台模式
  * @param[in]      none
  * @retval        
  * @attention      
  */
 gimbal_behaviour_e Return_gimbal_mode(void)
 {
      return gimbal_control.gimbal_behaviour;
 }
