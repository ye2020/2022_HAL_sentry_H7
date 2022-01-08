/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file 			chassis.c/h
 *
 * @brief 		底盘初始化设置
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

#include "SysInit.h"
#include "main.h"
#include "chassis_app.h"

osThreadId ChassisTaskHandle;
static FDCAN_TxHeaderTypeDef Txmessage;				//发送的信息
extern FDCAN_HandleTypeDef hfdcan1;                     // 句柄
extern FDCAN_HandleTypeDef hfdcan2;


/**
  * @brief      底盘初始化
  * @param[in]  
  * @retval     void
*/

void chassis_app_init(void)
{
	

		
	
//	CAN1_receive_callback = CAN1_chassis_receive;

	
	/*  创建底盘任务 */
  osThreadDef(ChassisTask, Chassis_Task, osPriorityRealtime, 0, 200);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

}


/**
	* @brief		can1发送函数
	* @param		传入参数： ESC_201~ESC_204 -> 代表同一电调上4个不同ID的电机
	*	@retval		none
  */
void Chassis_CAN_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    uint8_t Data[8];										//发送数据的数组

		Txmessage.Identifier = 0x200;												// 发送id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// 标识符类型 ,(标准帧)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// 指定将要传输的消息的帧类型。(数据帧)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// 数据长度 ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// 指定错误状态指示器。 (发送节点错误活跃)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// 指定发送的Tx帧是带位还是不带位率转换。(位率转换)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// 指定Tx帧将以经典或FD格式。(经典)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 指定帧开始时捕获的时间戳计数器值传播。(不存储Tx事件)
		Txmessage.MessageMarker = 0; 												// 指定复制到Tx Event FIFO元素中的消息标记用于识别Tx消息状态。
	

    Data[0] = (ESC_201 >> 8);
    Data[1] = ESC_201;
		Data[2] = (ESC_202>>8);
		Data[3] = ESC_202;
		Data[4] = (ESC_203>>8);
		Data[5] = ESC_203;
		Data[6] = (ESC_204>>8);
		Data[7] = ESC_204;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txmessage, Data);			//将一段数据通过 CAN 总线发送
}


/**
	* @brief		can2发送函数
	* @param		传入参数： ESC_201~ESC_204 -> 代表同一电调上4个不同ID的电机
	*	@retval		none
  */
void Chassis_CAN2_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    uint8_t Data[8];										//发送数据的数组

		Txmessage.Identifier = 0x200;												// 发送id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// 标识符类型 ,(标准帧)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// 指定将要传输的消息的帧类型。(数据帧)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// 数据长度 ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// 指定错误状态指示器。 (发送节点错误活跃)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// 指定发送的Tx帧是带位还是不带位率转换。(位率转换)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// 指定Tx帧将以经典或FD格式。(经典)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 指定帧开始时捕获的时间戳计数器值传播。(不存储Tx事件)
		Txmessage.MessageMarker = 0; 			

    Data[0] = (ESC_201 >> 8);
    Data[1] = ESC_201;
		Data[2] = (ESC_202>>8);
		Data[3] = ESC_202;
		Data[4] = (ESC_203>>8);
		Data[5] = ESC_203;
		Data[6] = (ESC_204>>8);
		Data[7] = ESC_204;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//将一段数据通过 CAN 总线发送

}



/* 底盘板CAN1遥控器数据发送 */
void CAN2_Chassis_RC_SetMsg(const RC_ctrl_t *can1_RC_send)
{
    uint8_t Data[8];										//发送数据的数组

		Txmessage.Identifier = 0x400;												// 发送id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// 标识符类型 ,(标准帧)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// 指定将要传输的消息的帧类型。(数据帧)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// 数据长度 ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// 指定错误状态指示器。 (发送节点错误活跃)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// 指定发送的Tx帧是带位还是不带位率转换。(位率转换)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// 指定Tx帧将以经典或FD格式。(经典)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 指定帧开始时捕获的时间戳计数器值传播。(不存储Tx事件)
		Txmessage.MessageMarker = 0; 		
	
		Data[0] = (unsigned char)(can1_RC_send->rc.ch[2] >> 8);
    Data[1] = (unsigned char)(can1_RC_send->rc.ch[2]);
		Data[2] = (unsigned char)(can1_RC_send->rc.ch[3] >> 8);
		Data[3] = (unsigned char)(can1_RC_send->rc.ch[3]);
		Data[4] = (unsigned char)(can1_RC_send->rc.ch[4] >> 8);
		Data[5] =	(unsigned char)(can1_RC_send->rc.ch[4]);
		Data[6] = (unsigned char)(can1_RC_send->rc.s[0]);
		Data[7] = (unsigned char)(can1_RC_send->rc.s[1]);
	
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//将一段数据通过 CAN 总线发送

	
}	

/* 底盘板发送yaw轴数据给云台板 */
void CAN2_Chassis_YAW_SetMsg(const motor_measure_t *can1_YAW_send)
{
    uint8_t Data[8];										//发送数据的数组

		Txmessage.Identifier = 0x5ff;												// 发送id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// 标识符类型 ,(标准帧)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// 指定将要传输的消息的帧类型。(数据帧)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// 数据长度 ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// 指定错误状态指示器。 (发送节点错误活跃)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// 指定发送的Tx帧是带位还是不带位率转换。(位率转换)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// 指定Tx帧将以经典或FD格式。(经典)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// 指定帧开始时捕获的时间戳计数器值传播。(不存储Tx事件)
		Txmessage.MessageMarker = 0; 			
	
    Data[0] = (unsigned char)(can1_YAW_send->position >> 8);
    Data[1] = (unsigned char)(can1_YAW_send->position);
    Data[2] = (unsigned char)(can1_YAW_send->speed    >> 8);
    Data[3] = (unsigned char)(can1_YAW_send->speed   );
    Data[4] = 0;
    Data[5] = 0;
    Data[6] = 0;
    Data[7] = 0;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//将一段数据通过 CAN 总线发送

}


/* 底盘通过can1发送yaw数据 */
void CAN1_Chassis_yaw_Setmsg(int16_t ESC_208)
{
  uint8_t Data[8];										//发送数据的数组

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
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txmessage, Data);			//将一段数据通过 CAN 总线发送

}

