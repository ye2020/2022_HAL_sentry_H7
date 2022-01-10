/**
******************************************************************************
* @file       gimbal_behaviour.c/h
* @brief      云台状态机。
******************************************************************************
*/


#include "gimbal_behaviour.h"
#include "SysInit.h"



/**************** 变量 *******************/ 


// 下云台遥控状态表
gimbal_behaviour_e gimbal_remote_control_Table[3][3] = 
{           /*右上*/                     /* 右下 */                          /* 右中 */
  /*左上*/  {GIMBAL_AUTOCONTROL,         GIMBAL_STANDBY,            GIMBAL_REMOTECONTROL_HIGH_SPEED   },
  /*左下*/  {GIMBAL_STANDBY,             GIMBAL_STANDBY,            GIMBAL_REMOTECONTROL_STOP_SHOOT   },
  /*左中*/  {GIMBAL_AUTOATTACK,          GIMBAL_STANDBY,            GIMBAL_REMOTECONTROL_LOW_SPEED    }
}; 

// 上云台遥控状态表
gimbal_behaviour_e above_gimbal_remote_control_Table[3][3] = 
{           /*右上*/                     /* 右下 */                          /* 右中 */
  /*左上*/  {GIMBAL_AUTOCONTROL,         GIMBAL_REMOTECONTROL_HIGH_SPEED,       GIMBAL_STANDBY   },
  /*左下*/  {GIMBAL_AUTOATTACK,    			 GIMBAL_STANDBY,                        GIMBAL_STANDBY   },
  /*左中*/  {GIMBAL_STANDBY,      			 GIMBAL_REMOTECONTROL_STOP_SHOOT,       GIMBAL_STANDBY   }
};    



float Gimbal_ch2 = 0.0f, Gimbal_ch3 = 0.0f;          // 云台电机受控量 ( ch2为ywa轴控制量，ch3为pitch轴)
static Shoot_WorkStatus_e Friction_wheel_mode = STOP_SHOOT; // 火控射速控制量
static Fire_WorkStatus_e Fire_mode = STOP_FIRE;             // 火控开火模式控制量

static uint16_t vision_status_count = 0;                    // 敌人消失计数
static uint16_t STANDBY_error_count = 0;										// 遥控通道ch[2],归零时会出现拨杆开关短暂错误


/**************** 函数声明 *******************/ 

static void Gimbal_AutoControl(gimbal_control_t *gimbal_autocontrol_f);           //  比赛模式(自瞄+巡逻)
static void Gimbal_Patrol(gimbal_control_t *gimbal_Patro_f);                      //  云台巡逻
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f);       //  云台遥控模式



/**
  * @brief         云台状态选择
  * @param[in]      fir_gimbal_behaviour_f
  * @retval         none
  * @attention
  */
void Gimbal_behaviour_mode_set(gimbal_control_t *fir_gimbal_behaviour_f)
{
    const char rc_sw1_lift    = (fir_gimbal_behaviour_f ->gimbal_RC ->rc.s[1] - 1);           // 遥控拨杆值减1
		const char rc_sw2_right 	= (fir_gimbal_behaviour_f ->gimbal_RC ->rc.s[0] - 1); 
    
    if(Get_appinit_status() == GIMBAL_APP)          
    fir_gimbal_behaviour_f->gimbal_behaviour = gimbal_remote_control_Table[rc_sw1_lift][rc_sw2_right];
    else if(Get_appinit_status() == CHASSIS_APP)
    fir_gimbal_behaviour_f->gimbal_behaviour = above_gimbal_remote_control_Table[rc_sw1_lift][rc_sw2_right];

    switch (fir_gimbal_behaviour_f->gimbal_behaviour)
    {
      /*  比赛模式 */
      case GIMBAL_AUTOCONTROL:
      {
        Gimbal_AutoControl(fir_gimbal_behaviour_f);
        break;
      }
      /*  初始化 */
      case GIMBAL_STANDBY:
      {
//				if((rc_sw1_lift != 1 && rc_sw2_right != 1) || STANDBY_error_count >= 5)   // 遥控通道ch[2],归零时会出现拨杆开关短暂错误，导致s1，s2会突然变成2
//				{																																					// 未查明原因，此处为一个治标不治本的补丁
					Gimbal_Stop(fir_gimbal_behaviour_f);                       //停止
					Remote_reload();                                           //摇杆量清零
//				}
//				else STANDBY_error_count++;
        break;
      }
      /* 自瞄模式 */
      case GIMBAL_AUTOATTACK:
      {
        Gimbal_AutoControl(fir_gimbal_behaviour_f);
        Friction_wheel_mode = STOP_SHOOT;
			  Fire_mode = STOP_FIRE;
        break;
      }
      /*  手控低射速 */
      case GIMBAL_REMOTECONTROL_LOW_SPEED:
      {
        Gimbal_RemoteControl(fir_gimbal_behaviour_f);									//遥控控制
        Friction_wheel_mode = LOW_SPEED;
        break;
      }
      /*  手控停火 */
      case GIMBAL_REMOTECONTROL_STOP_SHOOT:
      {
        Gimbal_RemoteControl(fir_gimbal_behaviour_f);									//遥控控制
        Friction_wheel_mode = STOP_SHOOT;
        break;
      }
      /*  手控高射速 */
      case GIMBAL_REMOTECONTROL_HIGH_SPEED:
      {
        Gimbal_RemoteControl(fir_gimbal_behaviour_f);									//遥控控制
        Friction_wheel_mode = HIGH_SPEED;
        break;
      }
      /*  云台无力 */
      case GIMBAL_STOP:
      {
        Remote_reload();       //摇杆量清零
        Gimbal_Stop(fir_gimbal_behaviour_f);
        break;
      }
      /* 遥控模式 */
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
  * @brief          云台自动状态
  * @param[in]      gimbal_auto_f
  * @retval         视觉控制量设置 并传入云台自动控制
  * @attention
  */
void Gimbal_Auto(gimbal_control_t *gimbal_auto_f)
 {
    if(gimbal_auto_f ->VisionStatus == Enemy_Disappear)
    {
      /* 实际位置 */
      gimbal_auto_f ->pitch_c.Auto_record_location = (gimbal_auto_f ->pitch_c.pitch_motor_measure ->actual_Position *360 /1024);
    }

    /*云台视觉控制量*/    //(y轴，p轴还没用卡尔曼)    
    gimbal_auto_f ->auto_c ->pitch_control_data = gimbal_auto_f->auto_c->auto_pitch_angle * 4.0f;
    gimbal_auto_f->auto_c->yaw_control_data     = gimbal_auto_f->auto_c->auto_yaw_angle * 4.0f;

    /* 限幅 pitch轴yaw轴角度限制 */
    gimbal_auto_f->auto_c->pitch_control_data = float_limit(gimbal_auto_f->auto_c->pitch_control_data, 5.0f*PITCH_ANGLE_LIMIT_UP , 25.0f*PITCH_ANGLE_LIMIT_DOWN);
    gimbal_auto_f->auto_c->yaw_control_data   = float_limit(gimbal_auto_f->auto_c->yaw_control_data , YAW_ANGLE_LIMIT , -YAW_ANGLE_LIMIT);

    Gimbal_Automatic_Work(gimbal_auto_f);
 }


/**
  * @brief          云台无力
  * @param[in]      gimbal_stop_f
  * @retval         none
  * @attention
  */
void Gimbal_Stop(gimbal_control_t *gimbal_stop_f)
{
    //控制量
    gimbal_stop_f->pitch_c.output = 0;
    gimbal_stop_f->yaw_c.output = 0;
    
    //自瞄控制量
    gimbal_stop_f->auto_c->pitch_control_data = 0;
    gimbal_stop_f->auto_c->yaw_control_data = 0;

	  Friction_wheel_mode = STOP_SHOOT;  
    Fire_mode           = STOP_FIRE;

//    Gimbal_ch3 = 0.0f;
//    Gimbal_ch2 = 0.0f;

}


/**
  * @brief          比赛模式(自瞄+巡逻)
  * @param[in]      gimbal_autocontrol_f
  * @retval         none
  * @attention
  */
static void Gimbal_AutoControl(gimbal_control_t *gimbal_autocontrol_f)
{
  /**************判断敌人出现或消失***************/
  {
    if(gimbal_autocontrol_f ->auto_c ->auto_pitch_angle == 0.0f && gimbal_autocontrol_f ->auto_c ->auto_yaw_angle == 0.0f)
    { 
       vision_status_count++;
    }
    else 
    {
       vision_status_count = 0;
       gimbal_autocontrol_f->VisionStatus = Enemy_Appear ;                 // 敌人出现
    }

    if ( vision_status_count >= ENEMY_DISAPPEAR_TIMES)
    {
      vision_status_count = ENEMY_DISAPPEAR_TIMES;
      gimbal_autocontrol_f->VisionStatus = Enemy_Disappear;		            //敌人消失
    }
  }

  /********************** 状态判断与选择 **********************/
  {
    if(gimbal_autocontrol_f->VisionStatus == Enemy_Disappear )      // 敌人消失
    {
      Gimbal_Patrol(gimbal_autocontrol_f);
      Fire_mode = STOP_FIRE;
    }

    if(gimbal_autocontrol_f->VisionStatus == Enemy_Appear)          // 敌人出现
    {
      Gimbal_Auto(gimbal_autocontrol_f);
			Fire_mode = FIRE;
    }
  }

}

/**
  * @brief          云台巡逻
  * @param[in]      *gimbal_Patro_f
  * @retval         none
  */
static uint32_t yaw_direction = RIGHT ;                 // yaw  轴方向
static uint32_t pitch_direction = PITCH_UP ;            // pitch轴方向
float Auto_Yaw_Angle_Target = 0.0f;                     // yaw  轴控制量
float Auto_Pitch_Angle_Target = 0.0f;                   // pitch轴控制量
 static void Gimbal_Patrol(gimbal_control_t *gimbal_Patro_f)
{

#if yaw_angle_limit	
	/*------------------  yaw轴角度限制 取消限制则可以yaw轴360----------------------------*/
	
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
				Auto_Yaw_Angle_Target -=0.2f;   //更改云台自动的速度
		}
		if(yaw_direction == RIGHT)
		{
				Auto_Yaw_Angle_Target +=0.2f;
		}
		
		/*P轴*/
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
  * @brief          云台遥控模式
  * @param[in]      gimbal_remotecontrol_f
  * @retval         none
  * @attention
  */
static void Gimbal_RemoteControl(gimbal_control_t *gimbal_remotecontrol_f)
{	
    Gimbal_ch2 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[0]) * RC_YAW_SPEED * 0.2f;     //Y轴位置环量累加   RC_YAW_SPEED
    Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f);                         //循环限幅，yaw角度限制     -180~180
    Gimbal_ch3 += (gimbal_remotecontrol_f->gimbal_RC->rc.ch[1]) * RC_PITCH_SPEED * 0.09f;  //P轴位置环量累加  RC_PITCH_SPEED

    Gimbal_ch3 = float_limit(Gimbal_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);   //pitch角度限制   0 ~ -85  (哨兵上负下正)

    if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] >= 500)
    {
        Fire_mode = FIRE;        //拨轮上  开火
    }
    else if (gimbal_remotecontrol_f->gimbal_RC->rc.ch[4] <= -500)
    {
        Fire_mode = BACK;       //拨轮下  退弹（反转）
    }
    else
    {
        Fire_mode = STOP_FIRE;  //拨轮中间  停止开火
    }

    Gimbal_Manual_Work(gimbal_remotecontrol_f , Gimbal_ch2 , Gimbal_ch3);
}




/**
  * @brief          返回摩擦轮模式
  * @param[in]      none
  * @retval         Friction_wheel_mode
  * @attention
  */
Shoot_WorkStatus_e Return_Friction_Wheel_Mode(void)
{
//	LOW_SPEED,   //低射速
//	HIGH_SPEED,  //高射速
//	STOP_SHOOT,  //停止
    if (Friction_wheel_mode == LOW_SPEED)
        return LOW_SPEED;

    if (Friction_wheel_mode == HIGH_SPEED)
        return HIGH_SPEED;

    if (Friction_wheel_mode == STOP_SHOOT)
        return STOP_SHOOT;

    return SHOOT_ERROR;
}



/**
  * @brief          返回发弹模式
  * @param[in]      none
  * @retval         Fire_mode
  * @attention
  */
Fire_WorkStatus_e Return_Fire_Mode(void)
{
//	FIRE,          //发射
//	AUTO_FIRE,     //自动发射
//	STOP_FIRE,     //停止发射
//	BACK,          //退弹
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

