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
static uint16_t STANDBY_chassis_error_count = 0;										// 遥控通道ch[2],归零时会出现拨杆开关短暂错误


 //底盘遥控选择状态表
 Chassis_mode_e chassis_remote_control_Table[RC_SW1_lift][RC_SW2_right] = 
{           /*右上*/                     /* 右下 */                          /* 右中 */
  /*左上*/  {CHASSIS_AUTO,              CHASSIS_STANDBY,             CHASSIS_REMOTECONTROL},
  /*左下*/  {CHASSIS_REMOTECONTROL,     CHASSIS_STANDBY,             CHASSIS_REMOTECONTROL},
  /*左中*/  {CHASSIS_AUTO,              CHASSIS_STANDBY,             CHASSIS_REMOTECONTROL}
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
				if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)   // 遥控通道ch[2],归零时会出现拨杆开关短暂错误，导致s1，s2会突然变成2
				{			
         chassis_behaviour_f -> chassis_mode = CHASSIS_INITIALIZE;  //状态设置为初始化
         Remote_reload(); 
				}
				else STANDBY_chassis_error_count++;												//摇杆量清零
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

			
    // 传入值处理
		if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_chassis_error_count >= 5)
		{
			chassis_set_remote(chassis_behaviour_f,Chassis_ch0,Chassis_ch1, Chassis_ch2);
		}
		if(chassis_behaviour_f->chassis_mode != CHASSIS_STANDBY)
		STANDBY_chassis_error_count = 0;

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


/**
  * @brief          底盘自动
  * @param[in]      *Chassis_Auto_f：底盘主结构体
  * @retval         none
  */
static void Chassis_Auto(chassis_control_t *Chassis_Auto_f)
{
    Chassis_ch2 = -CHASSIS_AUTO_SPPED;

}



/**
  * @brief          底盘走位
* @param[in]      *Chassis_Blocking_f:  底盘主结构体
  * @retval         none
  */
static void Chassis_Blocking(chassis_control_t *Chassis_Blocking_f)
{

}



/**
  * @brief          底盘遥控
  * @param[in]      *Chassis_Independent_f：底盘主结构体
  * @retval         none
  */
uint8_t back = 0 ,forward = 0 ;

 static void Chassis_RemoteControl(chassis_control_t *Chassis_RemoteControl_f)
{
    Chassis_ch0 = Chassis_RemoteControl_f->chassis_RC->rc.ch[0];
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

