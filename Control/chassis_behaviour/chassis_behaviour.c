/**
******************************************************************************
* @file       chassis_behaviour.c/h
* @brief      底盘状态机。
******************************************************************************
*/

#include "chassis_behaviour.h"
#include "SysInit.h"


/**************** 函数声明 *******************/ 
static void Chassis_Stop(chassis_control_t *Chassis_Stop_f);                              // 底盘无力状态函数
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f);                              // 底盘自动状态函数
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f);                      // 底盘走位状态函数
static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f);            // 底盘遥控状态函数

static uint8_t Get_Laser_Back(void);
static uint8_t Get_Laser_Forward(void);

/********************************************/

/****************** 变量 *********************/

 float Chassis_ch0 = 0.0f, Chassis_ch1 = 0.0f, Chassis_ch2 = 0.0f; //底盘电机受控量(ch0用于遥控模式的控制量，ch2用于自动模式的控制量)



 //底盘遥控选择状态表
 Chassis_mode_e chassis_remote_control_Table[RC_SW1_lift][RC_SW2_right] = 
{           /*右上*/                     /* 右下 */                          /* 右中 */
  /*左上*/  {CHASSIS_AUTO,              CHASSIS_REMOTECONTROL,             CHASSIS_REMOTECONTROL},
  /*左下*/  {CHASSIS_AUTO,     			CHASSIS_STANDBY,             	   CHASSIS_REMOTECONTROL},
  /*左中*/  {CHASSIS_AUTO,              CHASSIS_REMOTECONTROL,             CHASSIS_REMOTECONTROL}
};           


/*********************************************/


/**
  * @brief          底盘模式设置
  * @param[in]      *chassis_control_t：底盘主结构体
  * @retval         none
  */

void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour_f)
{
    const char rc_sw1_lift  		        = (chassis_behaviour_f ->chassis_RC ->rc.s[1] - 1);           // 遥控拨杆值减1
		const char rc_sw2_right 		        = (chassis_behaviour_f ->chassis_RC ->rc.s[0] - 1);           
    chassis_behaviour_f ->chassis_mode  = chassis_remote_control_Table[rc_sw1_lift][rc_sw2_right];    // 获取当前底盘状态

    /* 获取遥控数值并选择模式 */
    switch (chassis_behaviour_f -> chassis_mode)
    {
      /* 之前为无力状态 */
			case CHASSIS_STOP:
      {
        Remote_reload(); //摇杆量清零
        Chassis_Stop(chassis_behaviour_f);
				break;
      }
			/* 之前为待机状态 */
      case CHASSIS_STANDBY:
      {
//				if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)   // 遥控通道ch[2],归零时会出现拨杆开关短暂错误，导致s1，s2会突然变成2
				{			
         chassis_behaviour_f -> chassis_mode = CHASSIS_INITIALIZE;  //状态设置为初始化
         Remote_reload(); 
				}
//				else STANDBY_chassis_error_count++;												//摇杆量清零
         break;                                           
      }
			/*  自动模式 */
      case CHASSIS_AUTO:
      {
        Chassis_Auto(chassis_behaviour_f);
        break;
      }
      /* 遥控模式 */
      case CHASSIS_REMOTECONTROL:
      {
        Chassis_RemoteControl(chassis_behaviour_f);
        break;
      }
      /* 走位模式 */
      case CHASSIS_BLOCKING:
      {
        Chassis_Blocking(chassis_behaviour_f);
        break;
      }

			default:
			{
				chassis_behaviour_f->chassis_mode = CHASSIS_STANDBY; //进入待机模式
				break;
			}
    }

			
//    // 传入值处理
//	if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)
//	{
			chassis_set_remote(chassis_behaviour_f,Chassis_ch0,Chassis_ch1, Chassis_ch2);
//		}
//		if(chassis_behaviour_f->chassis_mode != CHASSIS_STANDBY)
//		STANDBY_chassis_error_count = 0;

} 


/**
  * @brief          底盘无力
  * @param[in]      *Chassis_Stop_f：底盘主结构体
  * @retval         none
  */
static void Chassis_Stop(chassis_control_t *Chassis_Stop_f)
{
    //*控制量
    Chassis_Stop_f->chassis_motor[0].output = 0;
    Chassis_Stop_f->chassis_motor[1].output = 0;
    Chassis_Stop_f->chassis_motor[2].output = 0;
    Chassis_Stop_f->chassis_motor[3].output = 0;

    Chassis_ch0 = Chassis_ch1 = Chassis_ch2 = 0;
/*     Fire_mode = STOP_FIRE;
   Friction_wheel_mode = STOP_SHOOT;  */
}


/*********************** 变量 *****************************/
static uint16_t outpost_HP = 0;													//前哨站血量

/**
  * @brief          底盘自动
  * @param[in]      *Chassis_Auto_f：底盘主结构体
  * @retval         none
  */
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f)
{

    if(automatic_Enemy_color() == Enemy_color_blue)       // 敌人为蓝方
      outpost_HP = 	referee_red_outpost_HP();							// 获取红方前哨站血量

    else if (automatic_Enemy_color() == Enemy_color_red)  // 敌方为红方
      outpost_HP =  referee_blue_outpost_HP();            // 获取蓝方前哨站血量

    if(outpost_HP >= 10)                                  // 己方前哨站血量较高
      Chassis_ch2 = CHASSIS_AUTO_SLOW_SPPED;

     else if(outpost_HP < 10) 
     {
       if((get_Enemy_status() == Enemy_Disappear) && (get_Enemy_status_from_above() == Enemy_Disappear))          // 敌人未出现
       {
          Chassis_Blocking(Chassis_Auto_f);             // 走位模式（控制方向和速度）

           if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))        // 边缘转向（优先级高于走位模式）
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


static uint16_t outpost_HP;													//前哨站血量
 uint8_t  Enemy_color;												//调试用
static uint16_t 		  speed_status_count = 0;							// 变速时间计时
static uint16_t 		  speed_status_count2 = 0;						// 变向时间计时
static int32_t  chassis_register1 = 5000;//5000;//360;						//速度记录值
static int32_t 	chassis_register2 = 0;							//速度记录值
static uint16_t 			speed_status_flag = 0;							// 边缘转向紧止随机 (1 -> 禁用 0 —> 可用)
static uint16_t 			forward_flag ,back_flag = 0;					// 随机模式1 强制转向标志位
#if (SPEED_MODE == 4)
uint16_t 		  speed_status_count3 = 0;									// 变向模式二时间计时
uint16_t 			chassis_flag = 0;													//变向模式二标志位
uint16_t  		speed_time_back_flag = SPEED_TIME_FLAG;			//计数时间1
uint16_t  		speed_time_forward_flag = SPEED_TIME_FLAG;	//计数时间1
#endif	
#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
uint16_t 			last_remain_HP = 600; 											//上次剩余血量
uint16_t 			changed_HP;																	// 变化血量		
uint16_t      speed_status_count4 = 0;									// 模式5计数值
#endif	


/**
  * @brief          底盘走位
* @param[in]      *Chassis_Blocking_f:  底盘主结构体
  * @retval       （ps： 这算法是好久之前赛前临时写的 没有任何封装 但效果还行 目前懒得去封装。留给后人doge） 
  */
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f)
{
#if   (SPEED_MODE == 1)										
            Chassis_ch2 = CHASSIS_AUTO_SPPED;
#elif (SPEED_MODE == 2 || SPEED_MODE == 3 || SPEED_MODE == 4)
						
						speed_status_count ++;
					
					if ((Get_Laser_Forward() == HAVE_THING) || (Get_Laser_Back() == HAVE_THING))	// 运动到边缘 ： 标志位置1 -> 一段时间内运动不收随机算法影响
					{
							speed_status_flag = 1;
						
							speed_status_count 	= 0;
							speed_status_count2 = 0; 
						
							#if (SPEED_MODE == 4)																																												// 防止频繁转向导致电机出错
							speed_status_count3 = 0; 
							#endif	
							#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			
							speed_status_count4 = 0;
							#endif												
					}
					
					  if(speed_status_count > SPEED_TIME_FLAG)
					{	
						speed_status_flag = 0;																																// 边缘转向标志位
						
						if(speed_status_flag == 0 )
						{
							speed_status_count = 0;															  															//计数值清零
							chassis_register1 = RNG_Get_RandomRange(4900,5200);//(4900,5000);											//产生4000~7000的随机数(4900 5000)
							Chassis_ch2 = chassis_register1;																										//赋值新的随机速度值
						}							
					}
						else
					{
						Chassis_ch2 = chassis_register1;												//速度值为上一次随机数的值
					}
#endif					
#if (SPEED_MODE == 3)
					speed_status_count2 ++;
			
					if(speed_status_count2 > SPEED_TIME_FLAG)									// 定时时间到 ： 产生随机数
					{
						speed_status_count2 =0;
						speed_status_flag = 0;																	
						chassis_register2 = RNG_Get_RandomRange(0,1);						// 产生0~1随机数
						
						if (chassis_register2 > 0)
						{
							forward_flag ++;
							back_flag = 0;
							if (forward_flag > 3 )
								chassis_register2 = 0;
							else
								chassis_register2 = 1;
						}
																																		// 控制变向的时间在 3 个周期内
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
					
					
					if(speed_status_flag == 0)																// 标志位为1时
				{
					if (chassis_register2 > 0)
					{	
						Chassis_Blocking_f->sign = GOFORWARD;												// 随机数大于1， 前进
					}
					
					if (chassis_register2 <= 0)															
					{	
            Chassis_Blocking_f->sign = GOBACK;														// 小于1, 后退
					}
		   	}	

#endif
#if (SPEED_MODE == 4)

				speed_status_count3 ++;
//				Chassis_ch2 = chassis_register1;													

        if((Get_Laser_Forward() == HAVE_THING) && (Get_Laser_Back() == NO_THING))			 
				{
					chassis_flag = 1; 																																		// 此时底盘后退开启新的周期						
					speed_time_forward_flag = SPEED_TIME_FLAG;																						// 不断重置另一种转态的计数上限
					speed_status_count3 = 0;
				}
				if((Get_Laser_Forward() == NO_THING) && (Get_Laser_Back() == HAVE_THING))
				{
					chassis_flag = 2;
					speed_time_back_flag = SPEED_TIME_FLAG;
					speed_status_count3 = 0;
				}
				
			
			if(speed_status_flag == 0)																															// 边缘转向禁用随机
			{
				if (chassis_flag == 1)																															
				{
					if(speed_status_count3 > speed_time_back_flag && Chassis_Blocking_f->sign == GOBACK)		// 计时器计数值到达 -> 往初始方向反向走
					{ 
						Chassis_Blocking_f->sign = GOFORWARD;
						speed_status_count3 = 0;
						speed_time_back_flag += SPEED_TIME_FLAG;																					// 计数值达到指定值 -> 指定值增加一个周期
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
#if (SPEED_MODE == 2 || SPEED_MODE == 4 || SPEED_MODE == 3)			          //挨打反向
				
				changed_HP = last_remain_HP - referee_remain_HP();										// 变化的血量
				last_remain_HP = referee_remain_HP();																	// 上次血量
				speed_status_count4++;
				
				if(speed_status_count4 > 0xFFF0)				                              // 防止计数溢出															
					speed_status_count4 = 0;
				
				if(changed_HP >= 10)
			{
					if(speed_status_count4 > SPEED_TIME_FLAG)                         // 避免频繁反向
				{
						if(speed_status_flag == 0)																			// 边缘转向禁用随机
						{	
							(Chassis_Blocking_f->sign == GOBACK)? (Chassis_Blocking_f->sign = GOFORWARD) : (Chassis_Blocking_f->sign = GOBACK);			// 挨打反向
							chassis_register1 = CHASSIS_BLOCKING_SPPED;
							
							speed_status_count = 0;
							speed_status_count2 = 0; 
							#if (SPEED_MODE == 4)																																												// 防止频繁转向导致电机出错
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
  * @brief          底盘遥控
  * @param[in]      *Chassis_Independent_f：底盘主结构体
  * @retval         none
  */
uint8_t back = 0 ,forward = 0 ;

 static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f)
{
    Chassis_ch0 = Chassis_RemoteControl_f->chassis_RC->rc.ch[2];
		back 		= Get_Laser_Back();
	  forward = Get_Laser_Forward();
}

//获取激光传感器的值
//距离内有东西为0, 没东西为1
//有消抖处理

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

