/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			can_2_receive.c/h
 *
 * @brief 	can2滤波器初始化以及底盘云台，can2接收
 *
 * @note  		
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */

#include "CAN_2_Receive.h"
#include "SysInit.h"

extern RC_ctrl_t rc_ctrl;
extern FDCAN_HandleTypeDef hfdcan1;                     // 句柄
extern FDCAN_HandleTypeDef hfdcan2;

//申明yaw轴3508电机变量
static motor_measure_t motor_yaw;
static VisionStatus_E  Enemy_status = Enemy_Disappear;			//敌人出现状态

//返回副yaw电机变量地址，通过指针方式获取原始数据

const motor_measure_t *Get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
VisionStatus_E  get_Enemy_status(void)
{
	return Enemy_status;
}


#if (gimbal_yaw_TO_chassis == 1)

static int16_t motor_yaw_output;

int16_t Get_Yaw_Gimbal_Motor_Output(void)
{
  return motor_yaw_output;
}
#endif
/**
	* @brief		can2滤波器配置
	* @param		none
	*	@retval		none
  */

void CAN2_filter_config(void)
{
  FDCAN_FilterTypeDef sFilterConfig1;

  sFilterConfig1.IdType = FDCAN_STANDARD_ID;
  sFilterConfig1.FilterIndex = 0;
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig1.FilterID1 = 0;
  sFilterConfig1.FilterID2 = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig1);

  sFilterConfig1.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig1.FilterIndex = 0;
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig1.FilterID1 = 0;
  sFilterConfig1.FilterID2 = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig1);   
  
 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); // 配置全局过滤器以拒绝所有不匹配的帧 
	

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // 在两个FDCAN实例上激活Rx FIFO 0新消息通知 

	
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);			// 溢满覆盖模式 
	HAL_FDCAN_Start(&hfdcan2);		
	

}


/**
	* @brief		底盘板can2的回调函数
	* @param		传入参数： CAN的句柄
	* @retval   none
  */
void chassis_can2_callback (FDCAN_HandleTypeDef *hcan)
{
  	FDCAN_RxHeaderTypeDef	rx_message;															//接收信息结构体
   	uint8_t Rx_Data[8];																					//接收的信息缓存的数组
	
	  if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK)	//读取接收的信息
	{	
		HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

        switch (rx_message.Identifier)
		{	
        case 0x4ff:
        {
            Enemy_status = (Rx_Data[7]);

            break;
        }

			 default:
        {
            break;
        }
    }
  }

}


/**
	* @brief		云台板can2的回调函数
	* @param		传入参数： CAN的句柄
	* @retval   none
  */
void gimbal_can2_callback (FDCAN_HandleTypeDef *hcan)
{
  	FDCAN_RxHeaderTypeDef	rx_message;															//接收信息结构体
   	uint8_t Rx_Data[8];																					//接收的信息缓存的数组
	
	  if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK)	//读取接收的信息
	{	
		HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    switch (rx_message.Identifier)
		{	
        /*云台电机*/
        case CAN_YAW_MOTOR_ID:
        {
					  motor_yaw.position 					= (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
            Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //计算yaw电机的真实码盘值
            motor_yaw.yaw_angle				  = motor_yaw.actual_Position * 360 / 8192 / YAW_RATIO;
            motor_yaw.speed 				 		= (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);
					
					break;
				}

				 case 0x400:
        {
//			Safecheck_dog.RC_Receive_Flag = 1;
            rc_ctrl.rc.ch[2] = (uint16_t)((Rx_Data[0] << 8) | Rx_Data[1]);
            rc_ctrl.rc.ch[3] = (uint16_t)((Rx_Data[2] << 8) | Rx_Data[3]);
            rc_ctrl.rc.ch[4] = (uint16_t)((Rx_Data[4] << 8) | Rx_Data[5]);
            rc_ctrl.rc.s[0] =  (Rx_Data[6]);
            rc_ctrl.rc.s[1] =  (Rx_Data[7]);
			
            break;
        }

        /* 由云台板发给底盘板的yaw轴电机输出值 */
				#if (gimbal_yaw_TO_chassis == 1)
        case 0x500:
        {
          motor_yaw_output = (uint16_t)((Rx_Data[6] << 8) | Rx_Data[7]);
          break;
        }
				#endif

			 default:
        {
            break;
        }
    }
  }

}

