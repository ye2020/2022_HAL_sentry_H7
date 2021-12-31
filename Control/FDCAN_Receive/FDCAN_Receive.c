#include "main.h"
#include "FDCAN_Receive.h"


//�������̵������ static
motor_measure_t motor_chassis[4];

FDCAN_TxHeaderTypeDef TxHeader1;


extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;


void fdcan1_config(void)
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
  
 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); // ����ȫ�ֹ������Ծܾ����в�ƥ���֡ 
	

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // ������FDCANʵ���ϼ���Rx FIFO 0����Ϣ֪ͨ 

	
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);			// ��������ģʽ 
	HAL_FDCAN_Start(&hfdcan1);																														// ����FDCAN
}

void fdcan2_config(void)
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
  
 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); // ����ȫ�ֹ������Ծܾ����в�ƥ���֡ 
	

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // ������FDCANʵ���ϼ���Rx FIFO 0����Ϣ֪ͨ 

	
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);			// ��������ģʽ 
	HAL_FDCAN_Start(&hfdcan2);		
	
}

uint32_t ava=0;
void Chassis_CAN_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
	uint8_t Data[8];

	TxHeader1.Identifier = 0x200;												// ����id
	TxHeader1.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
	TxHeader1.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
	TxHeader1.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
	TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
	TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
	TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
	TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
	TxHeader1.MessageMarker = 0; 												// ָ�����Ƶ�Tx Event FIFOԪ���е���Ϣ�������ʶ��Tx��Ϣ״̬��
	
	Data[0] = (ESC_201 >> 8);
	Data[1] = ESC_201;
	Data[2] = (ESC_202>>8);
	Data[3] = ESC_202;
	Data[4] = (ESC_203>>8);
	Data[5] = ESC_203;
	Data[6] = (ESC_204>>8);
	Data[7] = ESC_204;
//	Data[8] = ESC_201;
//	Data[9] = ESC_201;
//	Data[10] = ESC_201;
//	Data[11] = ESC_201;
//	Data[12] = ESC_201;
//	Data[13] = ESC_201;
//	Data[14] = ESC_201;
//	Data[15] = ESC_201;
	ava =  hfdcan1.msgRam.EndAddress - SRAMCAN_BASE;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, Data);			//��һ������ͨ�� CAN ���߷���

}


void Chassis_CAN2_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
	uint8_t Data[8];

	TxHeader1.Identifier = 0x200;												// ����id
	TxHeader1.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
	TxHeader1.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
	TxHeader1.DataLength  = FDCAN_DLC_BYTES_8;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
	TxHeader1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		// ָ������״ָ̬ʾ���� (���ͽڵ�����Ծ)
	TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;   					// ָ�����͵�Tx֡�Ǵ�λ���ǲ���λ��ת����(λ��ת��)
	TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;							// ָ��Tx֡���Ծ����FD��ʽ��(����)
	TxHeader1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	// ָ��֡��ʼʱ�����ʱ���������ֵ������(���洢Tx�¼�)
	TxHeader1.MessageMarker = 0; 												// ָ�����Ƶ�Tx Event FIFOԪ���е���Ϣ�������ʶ��Tx��Ϣ״̬��
	
	Data[0] = (ESC_201 >> 8);
	Data[1] = ESC_201;
	Data[2] = (ESC_202>>8);
	Data[3] = ESC_202;
	Data[4] = (ESC_203>>8);
	Data[5] = ESC_203;
	Data[6] = (ESC_204>>8);
	Data[7] = ESC_204;
//	Data[8] = ESC_201;
//	Data[9] = ESC_201;
//	Data[10] = ESC_201;
//	Data[11] = ESC_201;
//	Data[12] = ESC_201;
//	Data[13] = ESC_201;
//	Data[14] = ESC_201;
//	Data[15] = ESC_201;
	
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader1, Data);			//��һ������ͨ�� CAN ���߷���

	
}

/**
	* @brief		HAl��can1�Ļص�����
	* @param		��������� CAN�ľ��
	* @retval   none
  */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
/*     CAN_RxHeaderTypeDef	Rxmessage;	
 */    
    if(hfdcan == &hfdcan1)
				CAN1_chassis_receive(hfdcan);
		if(hfdcan == &hfdcan2)
				CAN2_chassis_receive(hfdcan);
////			if(CAN1_receive_callback != NULL)
////					CAN1_receive_callback(hcan)	;               
//		
//	
}


 void CAN1_chassis_receive(FDCAN_HandleTypeDef *hcan)
 {
	 FDCAN_RxHeaderTypeDef	rx_message;															//������Ϣ�ṹ��

   	uint8_t Rx_Data[8];																					//���յ���Ϣ���������
	 
	 	if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK)
		{
				HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			
				switch(rx_message.Identifier)
				{
				        /*���̵��*/
						case 0x201:
					{
						motor_chassis[0].position 		 = (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
						motor_chassis[0].speed   			 = (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);
						motor_chassis[0].given_current = (uint16_t)((Rx_Data[4] << 8 | Rx_Data[5]));
						motor_chassis[0].temperate		 =	Rx_Data[6];
						
						break;
					}
					
				}
			
		}			

	
 }

void CAN2_chassis_receive(FDCAN_HandleTypeDef *hcan)
{
	 FDCAN_RxHeaderTypeDef	rx_message;															//������Ϣ�ṹ��

   	uint8_t Rx_Data[8];																					//���յ���Ϣ���������
	 
	 	if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK)
		{
				HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			
				switch(rx_message.Identifier)
				{
				        /*���̵��*/
						case 0x201:
					{
						motor_chassis[1].position 		 = (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
						motor_chassis[1].speed   			 = (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);
						motor_chassis[1].given_current = (uint16_t)((Rx_Data[4] << 8 | Rx_Data[5]));
						motor_chassis[1].temperate		 =	Rx_Data[6];
						
						break;
					}
					
				}
			
		}			

	
}
 