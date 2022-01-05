/**
  *****************************东莞理工学院ACE实验室 *****************************
  * @file       chassis_task.c/h
  * @brief      完成底盘控制任务
  * @note       合并版本
  * @history    
  *
  @verbatim   v1.0
  ==============================================================================
  
  ==============================================================================
  @endverbatim
  *****************************东莞理工学院ACE实验室 *****************************
	*/
	
#include "Task_Chassis.h"
#include "SysInit.h"
#include "rng.h"
/**************** 函数声明 *******************/ 

static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f);       // 底盘遥控模式选择
static void chassis_controlwork(chassis_control_t *chassis_control_f);                  // 底盘主要任务控制函数
static void chassis_init(chassis_control_t *chassis_move_init_f);                       // 底盘初始化
static void chassis_data_update(chassis_control_t *chassis_data_update_f);              // 底盘数据更新
static void chassis_pid_calc(chassis_control_t *chassis_pid_f);                         // pid 计算       
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2);      // 底盘加速度限制斜坡函数
static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f);											// 底盘发送数据到云台


/********************************************/
//底盘控制数据 static
 chassis_control_t chassis_control;


void Chassis_Task(void const * argument)
	{
	    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);	
		
    /*底盘初始化*/
    chassis_init(&chassis_control);

		while(1)
		{
			
      /* 底盘控制 */
      chassis_controlwork(&chassis_control);


   #if chassis_using
			LEDE2(0);
    // 底盘发送电流值
      Chassis_CAN_Send_Msg(chassis_control.chassis_motor[0].output,
													 chassis_control.chassis_motor[1].output,
													 chassis_control.chassis_motor[2].output,
													 chassis_control.chassis_motor[3].output  );	
  #endif

  #if (gimbal_yaw_TO_chassis == 1)
    
      CAN1_Chassis_yaw_Setmsg(Get_Yaw_Gimbal_Motor_Output());

  #endif
			//检测周期
		vTaskDelay(CHASSIS_CONTROL_TIME);
		
		}                                                                                                                            
		
	}
	

	
/**
  * @brief          底盘遥控模式选择
  * @param[in]      *chassis_mode_choose_f：底盘主结构体
  * @retval         none
  */
static void chassis_remote_mode_choose(chassis_control_t *chassis_mode_choose_f)
{
    //判断是什么模式
    chassis_behaviour_mode_set(chassis_mode_choose_f);
}



/**
  * @brief          底盘主要任务控制函数
  * @param[in]      *chassis_control_f：底盘主结构体
  * @retval         none
  */
static void chassis_controlwork(chassis_control_t *chassis_control_f)
{
    // 检查遥控器数值是否正确
    RC_data_is_error();

    // 底盘数据更新
    chassis_data_update(chassis_control_f);

    // 遥控器模式状态设置
    chassis_remote_mode_choose(chassis_control_f);

    //底盘控制PID计算
    chassis_pid_calc(chassis_control_f);

    // 底盘发送数据给云台
		Chassis_to_Gimbal(chassis_control_f);

}


/**
  * @brief          底盘数据初始化
  * @param[in]      *chassis_move_init_f：底盘主结构体
  * @retval         none
  */
static void chassis_init(chassis_control_t *chassis_move_init_f)
{
  static uint8_t i;         // 选择电机

      /*--------------------获取指针--------------------*/
    {
            //返回遥控器控制变量，通过指针传递方式传递信息
        chassis_move_init_f->chassis_RC = get_remote_control_point();
            //获取底盘指针
                for (i = 0; i < 4; i++)
        {
            chassis_move_init_f->chassis_motor[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
        }

           
      #if (gimbal_yaw_TO_chassis == 1)
      // yaw轴数据 （用来发送给底盘计算）
      chassis_move_init_f->yaw_motor_measure = Chassis_Get_Yaw_Gimbal_Motor_Measure_Point();
      #endif 

    }
    
    /*--------------------检查遥控器数值是否正确--------------------*/
    RC_data_is_error();
		
		
    /*--------------------初始化低通滤波--------------------*/
    {
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
        first_order_filter_init(&(chassis_move_init_f->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);
    }

     /*--------------------初始化底盘pid--------------------*/
    // 真正用到的就motor[0], 这里还是初始化四个电机，包括后面调用和数据更新还是会四个电机一起，防止可能之后底盘电机数量增加。（ps:就是懒而已   _(:з」∠)_   ）
    {
        //底盘移动pid
        pid_init(&chassis_move_init_f->chassis_speed_pid[0], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[1], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[2], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);
        pid_init(&chassis_move_init_f->chassis_speed_pid[3], CHASSIS_MOTOR_PID_Kp, CHASSIS_MOTOR_PID_Ki, CHASSIS_MOTOR_PID_Kd, 0, 0);

        //
        pid_init(&chassis_move_init_f->chassis_location_pid, CHASSIS_LOCATION_PID_P, CHASSIS_LOCATION_PID_I, CHASSIS_LOCATION_PID_D, 0, 0);
    }

    //底盘开机状态为停止 (注意必须在获取指针以后)
    Chassis_Task_OFF(2);;

    // 开机时初始化前进方向
    chassis_move_init_f->sign = GOFORWARD;

    // 初次进入更新数据
    chassis_data_update(chassis_move_init_f);

    
}


/**
  * @brief          底盘发送数据到云台（yaw轴电机换算码盘值，yaw轴电机速度值，遥控值）
  * @param[in]      *chassis_setmsg_f：底盘主结构体
  * @retval         none
  */
static int send_sign = 0; //顺序发送遥控值

static void Chassis_to_Gimbal(chassis_control_t *chassis_setmsg_f)
{
	if ((chassis_setmsg_f->chassis_mode != CHASSIS_STANDBY) || (chassis_setmsg_f->chassis_mode != CHASSIS_STOP)) //待机刷新数据
	{
      if (send_sign == 0)
    {
      CAN2_Chassis_RC_SetMsg(chassis_setmsg_f->chassis_RC); //否则为遥控器模式
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
  * @brief          底盘加速度限制斜坡函数
  * @param[in]      *ch0：
  *                 *ch1：
  *                 *ch2：
  * @retval         斜坡函数的最终目的就是让输入信号变得更加平缓，减少系统超调，从而优化系统的时间响应。
  */
static void Chassis_accelerated_Control(int16_t *ch0, int16_t *ch1, int16_t *ch2)
{
    static int16_t last_ch[3] = {0, 0, 0};
    int16_t temp[3];

    temp[0] = *ch0 - last_ch[0];
    temp[1] = *ch1 - last_ch[1];
    temp[2] = *ch2 - last_ch[2];

    if (chassis_control.chassis_mode == CHASSIS_AUTO) //自动模式
    {
        if (float_abs(temp[0]) > TRANSLATION_ACCELERAD)
            *ch0 = last_ch[0] + temp[0] / float_abs(temp[0]) * TRANSLATION_ACCELERAD;

        if (float_abs(temp[1]) > STRAIGHT_ACCELERAD)
            *ch1 = last_ch[1] + temp[1] / float_abs(temp[1]) * STRAIGHT_ACCELERAD;
    }

    if (chassis_control.chassis_mode == CHASSIS_REMOTECONTROL) //遥控模式
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
  * @brief          底盘数据更新
  * @param[in]      *chassis_data_update_f：底盘主结构体
  * @retval         none
  * @attention
  */
static void chassis_data_update(chassis_control_t *chassis_data_update_f)
{
    uint8_t i;

    //电机速度更新
    for (i = 0; i < 4; i++)
    {
        chassis_data_update_f->chassis_motor[i].speed = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->speed;
        chassis_data_update_f->chassis_motor[i].position = chassis_data_update_f->chassis_motor[i].chassis_motor_measure->position;
        chassis_data_update_f->chassis_motor[i].accel = chassis_data_update_f->chassis_speed_pid[i].Derror[0] * 500.0f;
    }
}


/**
  * @brief          底盘控制PID计算
  * @param[in]      *chassis_pid_f：底盘主结构体
  * @retval         none
  */
static void chassis_pid_calc(chassis_control_t *chassis_pid_f)
{
  // 速度环pid （传入参数：速度环pid结构体spid，目标速度值setSpeed，实际速度值actualSpeed ，电流限幅 current_limit）

	 chassis_pid_f->chassis_motor[0].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[0]), chassis_pid_f->chassis_motor[0].speed_set, chassis_pid_f->chassis_motor[0].speed, M3508_MAX_OUTPUT_CURRENT); //M3508_MAX_OUTPUT_CURRENT
   chassis_pid_f->chassis_motor[1].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[1]), chassis_pid_f->chassis_motor[1].speed_set, chassis_pid_f->chassis_motor[1].speed, M3508_MAX_OUTPUT_CURRENT); //最大16000左右 MAX_MOTOR_CAN_OUTPUT
   chassis_pid_f->chassis_motor[2].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[2]), chassis_pid_f->chassis_motor[2].speed_set, chassis_pid_f->chassis_motor[2].speed, M3508_MAX_OUTPUT_CURRENT);
   chassis_pid_f->chassis_motor[3].output = Rmmotor_Speed_control(&(chassis_pid_f->chassis_speed_pid[3]), chassis_pid_f->chassis_motor[3].speed_set, chassis_pid_f->chassis_motor[3].speed, M3508_MAX_OUTPUT_CURRENT);



}


/**
  * @brief          底盘控制量设置
  * @param[in]      *chassis_set_f：底盘主结构体
  *                 ch0：处理后的ch0的数值
  *                 ch1：处理后的ch1的数值
  *                 ch2：处理后的ch2的数值
  * @retval         none
  */
 void chassis_set_remote(chassis_control_t *chassis_set_f, int16_t ch0, int16_t ch1, int16_t ch2)
{
      /*输入量斜坡函数处理*/
    Chassis_accelerated_Control(&ch0, &ch1, &ch2);

        //一阶低通滤波计算
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vx), -ch0);
    first_order_filter(&(chassis_set_f->LowFilt_chassis_vy), ch1);

        /*底盘遥控模式*/
    if (chassis_set_f->chassis_mode == CHASSIS_REMOTECONTROL)
    {			
        chassis_set_f->speed_x_set =5.0f * ch0;
        chassis_set_f->chassis_motor[0].speed_set = chassis_set_f->speed_x_set;
    }
        /* 待机模式 */
    if (chassis_set_f->chassis_mode == CHASSIS_STANDBY)
    {

        chassis_set_f->speed_x_set = 0.0f * ch0;
			
        chassis_set_f->chassis_motor[0].speed_set = chassis_set_f->speed_x_set;
    }
        /*底盘自动模式*/
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
  * @brief          底盘全部关闭
  * @param[in]      options:  0:清除底盘控制量  1:禁止底盘   2:禁止底盘加yaw轴和拨弹轮
  * @retval         none
  * @attention
  */
void Chassis_Task_OFF(uint8_t options)
{
    //清除底盘控制量
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

