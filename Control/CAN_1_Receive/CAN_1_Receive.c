/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			can_1_receive.c/h
 *
 * @brief 	滤波器初始化以及底盘云台，can1接收
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

#include "CAN_1_Receive.h"
#include "main.h"
#include "SysInit.h"

extern FDCAN_HandleTypeDef hfdcan1; // 句柄
extern FDCAN_HandleTypeDef hfdcan2;

void (*CAN1_receive_callback)(FDCAN_HandleTypeDef *);
void (*CAN2_receive_callback)(FDCAN_HandleTypeDef *);

/*--------------------变量-----------------------*/

#if (gimbal_yaw_TO_chassis == 1)
//申明yaw轴3508电机变量
static motor_measure_t motor_yaw;

//返回副yaw电机变量地址，通过指针方式获取原始数据
motor_measure_t *Chassis_Get_Yaw_Gimbal_Motor_Measure_Point(void)
{
  return &motor_yaw;
}

#endif

//申明底盘电机变量 static
motor_measure_t motor_chassis[4];
//申明拨弹电机变量
static motor_measure_t motor_fire_A;
static motor_measure_t motor_fire_B;
//申明pitch轴电机变量
static motor_measure_t motor_pitch;
//申明上云台pitch轴电机变量
static motor_measure_t motor_Above_pitch;
//申明上云台yaw轴电机变量
static motor_measure_t motor_Above_yaw;

//电机数据读取
#define get_motor_M3508(ptr, rx_message)                                              \
  {                                                                                   \
    (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
    (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);    \
  }

//返回拨弹电机变量地址，通过指针方式获取原始数据
const motor_measure_t *Get_Fire_MotorA_Measure_Point(void)
{
  return &motor_fire_A;
}
const motor_measure_t *Get_Fire_MotorB_Measure_Point(void)
{
  return &motor_fire_B;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)]; //(i & 0x03)
}

//返回pitch电机变量地址，通过指针方式获取原始数据
motor_measure_t *Get_Pitch_Gimbal_Motor_Measure_Point(void)
{
  return &motor_pitch;
}

//返回上云台pitch电机变量地址，通过指针方式获取原始数据
motor_measure_t *Get_Pitch_AboveGimbal_Motor_Measure_Point(void)
{
  return &motor_Above_pitch;
}

//返回上云台yaw电机变量地址，通过指针方式获取原始数据=
const motor_measure_t *Get_Yaw_AboveGimbal_Motor_Measure_Point(void)
{
  return &motor_Above_yaw;
}
/**
 * @brief		滤波器配置
 * @param		none
 *	@retval		none
 */

void CAN1_filter_config(void)
{
  FDCAN_FilterTypeDef sFilterConfig1;

  sFilterConfig1.IdType = FDCAN_STANDARD_ID;
  sFilterConfig1.FilterIndex = 0;
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig1.FilterID1 = 0;
  sFilterConfig1.FilterID2 = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1);

  sFilterConfig1.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig1.FilterIndex = 0;
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig1.FilterID1 = 0;
  sFilterConfig1.FilterID2 = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); // 配置全局过滤器以拒绝所有不匹配的帧

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 在两个FDCAN实例上激活Rx FIFO 0新消息通知

  HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE); // 溢满覆盖模式
  HAL_FDCAN_Start(&hfdcan1);                                                          // 开启FDCAN
}

/**
 * @brief		HAl库can1的回调函数
 * @param		传入参数： CAN的句柄
 * @retval   none
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  /*     CAN_RxHeaderTypeDef	Rxmessage;
   */
  //    if(hfdcan == &hfdcan1)
  //				CAN1_chassis_receive(&hfdcan1);
  //		if(hfdcan == &hfdcan2)
  //				CAN2_chassis_receive(&hfdcan1);
  //			if(CAN1_receive_callback != NULL)
  //					CAN1_receive_callback(hcan)	;
  if (hfdcan == &hfdcan1)
  {
    if (CAN1_receive_callback != NULL)
      CAN1_receive_callback(&hfdcan1);
  }
  if (hfdcan == &hfdcan2)
    if (CAN2_receive_callback != NULL)
      CAN2_receive_callback(&hfdcan2);
}

/*************************************can1接收处理函数*************************************/

/**
 * @brief		底盘板can1的回调函数
 * @param		传入参数： CAN的句柄
 * @retval   none
 */
void chassis_can1_callback(FDCAN_HandleTypeDef *hcan)
{
  FDCAN_RxHeaderTypeDef rx_message; //接收信息结构体
  uint8_t Rx_Data[8];               //接收的信息缓存的数组

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK) //读取接收的信息
  {
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断

    switch (rx_message.Identifier)
    {
    case 0x201:
    {
      motor_chassis[0].position = (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
      motor_chassis[0].speed = (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);
      motor_chassis[0].given_current = (uint16_t)((Rx_Data[4] << 8 | Rx_Data[5]));
      motor_chassis[0].temperate = Rx_Data[6];

      break;
    }

    case CAN_TRIGGER_MOTORA_ID:
    {
      motor_fire_A.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
      Motor_Actual_Position(&motor_fire_A, Sec_YAW_RATIO, 8192);
      motor_fire_A.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

      break;
    }

    case CAN_PIT_MOTOR_ID:
    {
      motor_Above_pitch.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));
      motor_Above_pitch.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
      Motor_Actual_Position(&motor_Above_pitch,1,8192);
      motor_Above_pitch.pitch_angle = ((motor_Above_pitch.actual_Position * 360 / (8192) / 1.5 / 19) - 47);

    }
    default:
      break;
    }
  }
}

/**
 * @brief		云台板can1的回调函数
 * @param		传入参数： CAN的句柄
 * @retval   none
 */
void gimbal_can1_callback(FDCAN_HandleTypeDef *hcan)
{
  FDCAN_RxHeaderTypeDef rx_message; //接收信息结构体
  uint8_t Rx_Data[8];               //接收的信息缓存的数组

  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK) //读取接收的信息
  {
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断

    switch (rx_message.Identifier)
    {
#if (pitch_angle_position == 1) // 使用编码器

    /* 编码器 */
    case 0x01:
    {

      motor_pitch.position = ((uint16_t)(Rx_Data[6] << 24) | (uint16_t)(Rx_Data[5] << 16) | (uint16_t)(Rx_Data[4] << 8) | (uint16_t)(Rx_Data[3]));

      if (motor_pitch.position >= 0 && motor_pitch.position <= 150)
      {
        motor_pitch.position += 884;
      }
      else
        motor_pitch.position -= 140;
      motor_pitch.actual_Position = motor_pitch.position; //实际值减去中间值
      motor_pitch.pitch_angle = -(motor_pitch.actual_Position * 360 / 1024 / PITCH_GR - Pitch_Middle_Angle);
      break;
    }
#endif
      /* pitch轴电机 */
    case CAN_PIT_MOTOR_ID:
    {

      motor_pitch.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

#if (pitch_angle_position == 0)
      motor_pitch.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
      Motor_Actual_Position(&motor_pitch, 1.5 * 19, 8192);
      motor_pitch.pitch_angle = ((motor_pitch.actual_Position * 360 / (8192) / 1.5 / 19) - 47);

#endif

      break;
    }

/* 底盘接收can1云台电机的数据 */
#if (gimbal_yaw_TO_chassis == 1)
    case 0x208:
    {
      motor_yaw.position = (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
      Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //计算yaw电机的真实码盘值
      motor_yaw.yaw_angle = motor_yaw.actual_Position * 360 / 8192 / YAW_RATIO;
      motor_yaw.speed = (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);

      break;
    }

#endif
      /* 拨弹电机A */
    case CAN_TRIGGER_MOTORA_ID:
    {
      motor_fire_A.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
      Motor_Actual_Position(&motor_fire_A, Sec_YAW_RATIO, 8192);
      motor_fire_A.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

      break;
    }

      /* 拨弹电机A */
    case CAN_TRIGGER_MOTORB_ID:
    {
      motor_fire_B.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
      Motor_Actual_Position(&motor_fire_B, Sec_YAW_RATIO, 8192);
      motor_fire_B.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

      break;
    }

    default:
    {
      break;
    }
    }
  }
}

// void CAN1_chassis_receive(FDCAN_HandleTypeDef *hcan)
//{
//   uint8_t appinit = Get_appinit_status(); // 获取初始化情况
//   FDCAN_RxHeaderTypeDef rx_message;       //接收信息结构体

//  uint8_t Rx_Data[8]; //接收的信息缓存的数组

//  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK) //读取接收的信息
//  {
//    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 启动中断

//    switch (rx_message.Identifier)
//    {
//#if (appinit == CHASSIS_APP) //初始化为底盘板
//    /*底盘电机*/
//    case 0x201:
//    {
//      motor_chassis[0].position = (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
//      motor_chassis[0].speed = (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);
//      motor_chassis[0].given_current = (uint16_t)((Rx_Data[4] << 8 | Rx_Data[5]));
//      motor_chassis[0].temperate = Rx_Data[6];

//      break;
//    }

//#elif (appinit == GIMBAL_APP)
//#if (pitch_angle_position == 1) // 使用编码器

//    /* 编码器 */
//    case 0x01:
//    {

//      motor_pitch.position = ((uint16_t)(Rx_Data[6] << 24) | (uint16_t)(Rx_Data[5] << 16) | (uint16_t)(Rx_Data[4] << 8) | (uint16_t)(Rx_Data[3]));

//      if (motor_pitch.position >= 0 && motor_pitch.position <= 150)
//      {
//        motor_pitch.position += 884;
//      }
//      else
//        motor_pitch.position -= 140;
//      motor_pitch.actual_Position = motor_pitch.position; //实际值减去中间值
//      motor_pitch.pitch_angle = -(motor_pitch.actual_Position * 360 / 1024 / PITCH_GR - Pitch_Middle_Angle);
//      break;
//    }
//#endif
//      /* pitch轴电机 */
//    case CAN_PIT_MOTOR_ID:
//    {

//      motor_pitch.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

//#if (pitch_angle_position == 0)
//      motor_pitch.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
//      Motor_Actual_Position(&motor_pitch, 1.5 * 19, 8192);
//      motor_pitch.pitch_angle = ((motor_pitch.actual_Position * 360 / (8192) / 1.5 / 19) - 47);

//#endif

//      break;
//    }

///* 底盘接收can1云台电机的数据 */
//#if (gimbal_yaw_TO_chassis == 1)
//    case 0x208:
//    {
//      motor_yaw.position = (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
//      Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //计算yaw电机的真实码盘值
//      motor_yaw.yaw_angle = motor_yaw.actual_Position * 360 / 8192 / YAW_RATIO;
//      motor_yaw.speed = (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);

//      break;
//    }

//#endif
//      /* 拨弹电机A */
//    case CAN_TRIGGER_MOTORA_ID:
//    {
//      motor_fire_A.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
//      Motor_Actual_Position(&motor_fire_A, Sec_YAW_RATIO, 8192);
//      motor_fire_A.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

//      break;
//    }

//      /* 拨弹电机A */
//    case CAN_TRIGGER_MOTORB_ID:
//    {
//      motor_fire_B.position = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
//      Motor_Actual_Position(&motor_fire_B, Sec_YAW_RATIO, 8192);
//      motor_fire_B.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

//      break;
//    }
//#endif

//    default:
//    {
//      break;
//    }
//    }
//  }
//}
