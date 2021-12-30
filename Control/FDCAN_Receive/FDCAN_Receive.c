#include "main.h"
#include "FDCAN_Receive.h"


FDCAN_TxHeaderTypeDef TxHeader1;


extern FDCAN_HandleTypeDef hfdcan1;

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
  
  /* ����ȫ�ֹ������Ծܾ����в�ƥ���֡ */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
  /* Configure Rx FIFO 0 watermark to 2 */
  //HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 2);

  /* ������FDCANʵ���ϼ���Rx FIFO 0����Ϣ֪ͨ */
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);

  /* Configure and enable Tx Delay Compensation, required for BRS mode.
        TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
        TdcFilter default recommended value: 0 */
  HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
  HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

 // HAL_FDCAN_Start(&hfdcan1);
}

void Chassis_CAN_Send_Msg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
	uint8_t Data[16];

	TxHeader1.Identifier = 0x200;												// ����id
	TxHeader1.IdType     = FDCAN_STANDARD_ID;						// ��ʶ������ ,(��׼֡)
	TxHeader1.TxFrameType = FDCAN_DATA_FRAME;						// ָ����Ҫ�������Ϣ��֡���͡�(����֡)
	TxHeader1.DataLength  = FDCAN_DLC_BYTES_16;					// ���ݳ��� ([0,8],12,16,20,24,32,48,64)
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
	Data[8] = ESC_201;
	Data[9] = ESC_201;
	Data[10] = ESC_201;
	Data[11] = ESC_201;
	Data[12] = ESC_201;
	Data[13] = ESC_201;
	Data[14] = ESC_201;
	Data[15] = ESC_201;
	
	
	//HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader1, Data);			//��һ������ͨ�� CAN ���߷���
  HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &TxHeader1, Data, FDCAN_TX_BUFFER0);

	
	/* ���ͻ�������Ϣ */
  HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER0);	 

}
