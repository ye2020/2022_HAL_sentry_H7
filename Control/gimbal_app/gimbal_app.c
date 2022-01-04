/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			gimbal.c/h
 *
 * @brief 		云台初始化设置
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
 
#include "gimbal_app.h"
#include "SysInit.h"
#include "main.h"


/***************** 变量 ******************/
osThreadId GIMBALTASKHandle;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim8;
static FDCAN_TxHeaderTypeDef Txmessage;				//发送的信息

extern FDCAN_HandleTypeDef hfdcan1;                     // 句柄
extern FDCAN_HandleTypeDef hfdcan2;

/********************************************/
/**
  * @brief     云台初始化
  * @param[in]  
  * @retval     void
*/

void gimbal_app_init(void)
{

	//开启PWM输出
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
  osThreadDef(GIMBALTASK, GIMBAL_TASK, osPriorityRealtime, 0, 200);
  GIMBALTASKHandle = osThreadCreate(osThread(GIMBALTASK), NULL);

}


/*************************************can1发送*************************************/
/**********************************************************************************/
/* 底盘板CAN1发送到云台Y电机和供弹电机 | Y轴电机是208，P轴电机是207，供弹电机是205 206 */
void CAN1_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208)
{
	uint8_t Data[8];

	Txmessage.Identifier = 0x1ff;												// 发送id
	Txmessage.IdType     = FDCAN_STANDARD_ID;						// 标识符类型 ,(标准帧)
	Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// 指定将要传输的消息的帧类型。(数据帧)
	Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// 数据长度 ([0,8],12,16,20,24,32,48,64)
	Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// 指定错误状态指示器。 (发送节点错误活跃)
	Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// 指定发送的Tx帧是带位还是不带位率转换。(位率转换)
	Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// 指定Tx帧将以经典或FD格式。(经典)
	Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 指定帧开始时捕获的时间戳计数器值传播。(不存储Tx事件)
	Txmessage.MessageMarker = 0; 												// 指定复制到Tx Event FIFO元素中的消息标记用于识别Tx消息状态。
	
	Data[0] = (ESC_205 >> 8);
	Data[1] = ESC_205;
	Data[2] = (ESC_206>>8);
	Data[3] = ESC_206;
	Data[4] = (ESC_207>>8);
	Data[5] = ESC_207;
	Data[6] = (ESC_208>>8);
	Data[7] = ESC_208;

	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txmessage, Data);			//将一段数据通过 CAN 总线发送
}	

/* yaw轴接can2 */
void CAN2_yaw_Setmsg(int16_t ESC_208)
{
	uint8_t Data[8];

	Txmessage.Identifier = 0x1ff;												// 发送id
	Txmessage.IdType     = FDCAN_STANDARD_ID;						// 标识符类型 ,(标准帧)
	Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// 指定将要传输的消息的帧类型。(数据帧)
	Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// 数据长度 ([0,8],12,16,20,24,32,48,64)
	Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// 指定错误状态指示器。 (发送节点错误活跃)
	Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// 指定发送的Tx帧是带位还是不带位率转换。(位率转换)
	Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// 指定Tx帧将以经典或FD格式。(经典)
	Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 指定帧开始时捕获的时间戳计数器值传播。(不存储Tx事件)
	Txmessage.MessageMarker = 0; 		
	
	Data[0] = (unsigned char)(0);
	Data[1] = (unsigned char)(0);
	Data[2] = (unsigned char)(0);
	Data[3] = (unsigned char)(0);
	Data[4] = (unsigned char)(0);
	Data[5] = (unsigned char)(0);
	Data[6] = (ESC_208>>8);
	Data[7] =  ESC_208;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//将一段数据通过 CAN 总线发送
}

/* yaw轴接can1 将云台计算好的输出值发送给底盘*/
void CAN2_Gimbal_yaw_Setmsg(int16_t ESC_208)
{
    uint8_t Data[8];										//发送数据的数组
	
	Txmessage.Identifier = 0x500;												// 发送id
	Txmessage.IdType     = FDCAN_STANDARD_ID;						// 标识符类型 ,(标准帧)
	Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// 指定将要传输的消息的帧类型。(数据帧)
	Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// 数据长度 ([0,8],12,16,20,24,32,48,64)
	Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// 指定错误状态指示器。 (发送节点错误活跃)
	Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// 指定发送的Tx帧是带位还是不带位率转换。(位率转换)
	Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// 指定Tx帧将以经典或FD格式。(经典)
	Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 指定帧开始时捕获的时间戳计数器值传播。(不存储Tx事件)
	Txmessage.MessageMarker = 0; 												// 指定复制到Tx Event FIFO元素中的消息标记用于识别Tx消息状态。
	
	
	Data[0] = (unsigned char)(0);
	Data[1] = (unsigned char)(0);
	Data[2] = (unsigned char)(0);
	Data[3] = (unsigned char)(0);
	Data[4] = (unsigned char)(0);
	Data[5] = (unsigned char)(0);
	Data[6] = (ESC_208>>8);
	Data[7] =  ESC_208;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//将一段数据通过 CAN 总线发送

}
