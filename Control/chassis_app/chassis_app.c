/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			chassis.c/h
 *
 * @brief 		���̳�ʼ������
 *
 * @note  		
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 */

#include "SysInit.h"
#include "main.h"
#include "chassis_app.h"

osThreadId ChassisTaskHandle;
static FDCAN_TxHeaderTypeDef Txmessage;				//���͵���Ϣ
extern FDCAN_HandleTypeDef hfdcan1;                     // ���
extern FDCAN_HandleTypeDef hfdcan2;


/**
  * @brief      ���̳�ʼ��
  * @param[in]  
  * @retval     void
*/

void chassis_app_init(void)
{
	

		
	
//	CAN1_receive_callback = CAN1_chassis_receive;

	
	/*  ������������ */
  osThreadDef(ChassisTask, Chassis_Task, osPriorityRealtime, 0, 200);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

}


/**
	* @brief		can1���ͺ���
	* @param		��������� ESC_201~ESC_204 -> ����ͬһ�����4����ͬID�ĵ��
	*	@retval		none
  */
void Chassis_CAN_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    uint8_t Data[8];										//�������ݵ�����

		Txmessage.Identifier = 0x200;												// ����id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
		Txmessage.MessageMarker = 0; 												// ָ�����Ƶ�Tx Event FIFOԪ���е���Ϣ�������ʶ��Tx��Ϣ״̬��
	

    Data[0] = (ESC_201 >> 8);
    Data[1] = ESC_201;
		Data[2] = (ESC_202>>8);
		Data[3] = ESC_202;
		Data[4] = (ESC_203>>8);
		Data[5] = ESC_203;
		Data[6] = (ESC_204>>8);
		Data[7] = ESC_204;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���
}


/**
	* @brief		can2���ͺ���
	* @param		��������� ESC_201~ESC_204 -> ����ͬһ�����4����ͬID�ĵ��
	*	@retval		none
  */
void Chassis_CAN2_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    uint8_t Data[8];										//�������ݵ�����

		Txmessage.Identifier = 0x200;												// ����id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
		Txmessage.MessageMarker = 0; 			

    Data[0] = (ESC_201 >> 8);
    Data[1] = ESC_201;
		Data[2] = (ESC_202>>8);
		Data[3] = ESC_202;
		Data[4] = (ESC_203>>8);
		Data[5] = ESC_203;
		Data[6] = (ESC_204>>8);
		Data[7] = ESC_204;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���

}



/* ���̰�CAN1ң�������ݷ��� */
void CAN2_Chassis_RC_SetMsg(const RC_ctrl_t *can1_RC_send)
{
    uint8_t Data[8];										//�������ݵ�����

		Txmessage.Identifier = 0x400;												// ����id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
		Txmessage.MessageMarker = 0; 		
	
		Data[0] = (unsigned char)(can1_RC_send->rc.ch[2] >> 8);
    Data[1] = (unsigned char)(can1_RC_send->rc.ch[2]);
		Data[2] = (unsigned char)(can1_RC_send->rc.ch[3] >> 8);
		Data[3] = (unsigned char)(can1_RC_send->rc.ch[3]);
		Data[4] = (unsigned char)(can1_RC_send->rc.ch[4] >> 8);
		Data[5] =	(unsigned char)(can1_RC_send->rc.ch[4]);
		Data[6] = (unsigned char)(can1_RC_send->rc.s[0]);
		Data[7] = (unsigned char)(can1_RC_send->rc.s[1]);
	
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���

	
}	

/* ���̰巢��yaw�����ݸ���̨�� */
void CAN2_Chassis_YAW_SetMsg(const motor_measure_t *can1_YAW_send)
{
    uint8_t Data[8];										//�������ݵ�����

		Txmessage.Identifier = 0x5ff;												// ����id
		Txmessage.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
		Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
		Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
		Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
		Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
		Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
		Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
		Txmessage.MessageMarker = 0; 			
	
    Data[0] = (unsigned char)(can1_YAW_send->position >> 8);
    Data[1] = (unsigned char)(can1_YAW_send->position);
    Data[2] = (unsigned char)(can1_YAW_send->speed    >> 8);
    Data[3] = (unsigned char)(can1_YAW_send->speed   );
    Data[4] = 0;
    Data[5] = 0;
    Data[6] = 0;
    Data[7] = 0;

		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���

}


/* ����ͨ��can1����yaw���� */
void CAN1_Chassis_yaw_Setmsg(int16_t ESC_208)
{
  uint8_t Data[8];										//�������ݵ�����

	Txmessage.Identifier = 0x1ff;												// ����id
	Txmessage.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
	Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
	Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
	Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
	Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
	Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
	Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
	Txmessage.MessageMarker = 0; 				

  Data[0] = (unsigned char)(0);
	Data[1] = (unsigned char)(0);
	Data[2] = (unsigned char)(0);
	Data[3] = (unsigned char)(0);
	Data[4] = (unsigned char)(0);
	Data[5] = (unsigned char)(0);
	Data[6] = (ESC_208>>8);
	Data[7] =  ESC_208;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���

}

