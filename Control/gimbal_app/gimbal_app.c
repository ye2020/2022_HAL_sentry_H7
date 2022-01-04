/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			gimbal.c/h
 *
 * @brief 		��̨��ʼ������
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
 
#include "gimbal_app.h"
#include "SysInit.h"
#include "main.h"


/***************** ���� ******************/
osThreadId GIMBALTASKHandle;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim8;
static FDCAN_TxHeaderTypeDef Txmessage;				//���͵���Ϣ

extern FDCAN_HandleTypeDef hfdcan1;                     // ���
extern FDCAN_HandleTypeDef hfdcan2;

/********************************************/
/**
  * @brief     ��̨��ʼ��
  * @param[in]  
  * @retval     void
*/

void gimbal_app_init(void)
{

	//����PWM���
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
  osThreadDef(GIMBALTASK, GIMBAL_TASK, osPriorityRealtime, 0, 200);
  GIMBALTASKHandle = osThreadCreate(osThread(GIMBALTASK), NULL);

}


/*************************************can1����*************************************/
/**********************************************************************************/
/* ���̰�CAN1���͵���̨Y����͹������ | Y������208��P������207�����������205 206 */
void CAN1_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207, int16_t ESC_208)
{
	uint8_t Data[8];

	Txmessage.Identifier = 0x1ff;												// ����id
	Txmessage.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
	Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
	Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
	Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
	Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
	Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
	Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
	Txmessage.MessageMarker = 0; 												// ָ�����Ƶ�Tx Event FIFOԪ���е���Ϣ�������ʶ��Tx��Ϣ״̬��
	
	Data[0] = (ESC_205 >> 8);
	Data[1] = ESC_205;
	Data[2] = (ESC_206>>8);
	Data[3] = ESC_206;
	Data[4] = (ESC_207>>8);
	Data[5] = ESC_207;
	Data[6] = (ESC_208>>8);
	Data[7] = ESC_208;

	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���
}	

/* yaw���can2 */
void CAN2_yaw_Setmsg(int16_t ESC_208)
{
	uint8_t Data[8];

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
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���
}

/* yaw���can1 ����̨����õ����ֵ���͸�����*/
void CAN2_Gimbal_yaw_Setmsg(int16_t ESC_208)
{
    uint8_t Data[8];										//�������ݵ�����
	
	Txmessage.Identifier = 0x500;												// ����id
	Txmessage.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
	Txmessage.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
	Txmessage.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
	Txmessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
	Txmessage.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
	Txmessage.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
	Txmessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
	Txmessage.MessageMarker = 0; 												// ָ�����Ƶ�Tx Event FIFOԪ���е���Ϣ�������ʶ��Tx��Ϣ״̬��
	
	
	Data[0] = (unsigned char)(0);
	Data[1] = (unsigned char)(0);
	Data[2] = (unsigned char)(0);
	Data[3] = (unsigned char)(0);
	Data[4] = (unsigned char)(0);
	Data[5] = (unsigned char)(0);
	Data[6] = (ESC_208>>8);
	Data[7] =  ESC_208;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &Txmessage, Data);			//��һ������ͨ�� CAN ���߷���

}
