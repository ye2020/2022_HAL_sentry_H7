/**
 *****************************��ݸ��ѧԺACEʵ���� *****************************
 * @file 			can_2_receive.c/h
 *
 * @brief 	can2�˲�����ʼ���Լ�������̨��can2����
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

#include "CAN_2_Receive.h"
#include "SysInit.h"

extern RC_ctrl_t rc_ctrl;
extern FDCAN_HandleTypeDef hfdcan1;                     // ���
extern FDCAN_HandleTypeDef hfdcan2;

//����yaw��3508�������
static motor_measure_t motor_yaw;
static VisionStatus_E  Enemy_status = Enemy_Disappear;			//���˳���״̬

//���ظ�yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����

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
	* @brief		can2�˲�������
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
  
 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); // ����ȫ�ֹ������Ծܾ����в�ƥ���֡ 
	

  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // ������FDCANʵ���ϼ���Rx FIFO 0����Ϣ֪ͨ 

	
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);			// ��������ģʽ 
	HAL_FDCAN_Start(&hfdcan2);		
	

}


/**
	* @brief		���̰�can2�Ļص�����
	* @param		��������� CAN�ľ��
	* @retval   none
  */
void chassis_can2_callback (FDCAN_HandleTypeDef *hcan)
{
  	FDCAN_RxHeaderTypeDef	rx_message;															//������Ϣ�ṹ��
   	uint8_t Rx_Data[8];																					//���յ���Ϣ���������
	
	  if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK)	//��ȡ���յ���Ϣ
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
	* @brief		��̨��can2�Ļص�����
	* @param		��������� CAN�ľ��
	* @retval   none
  */
void gimbal_can2_callback (FDCAN_HandleTypeDef *hcan)
{
  	FDCAN_RxHeaderTypeDef	rx_message;															//������Ϣ�ṹ��
   	uint8_t Rx_Data[8];																					//���յ���Ϣ���������
	
	  if(HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK)	//��ȡ���յ���Ϣ
	{	
		HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    switch (rx_message.Identifier)
		{	
        /*��̨���*/
        case CAN_YAW_MOTOR_ID:
        {
					  motor_yaw.position 					= (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
            Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //����yaw�������ʵ����ֵ
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

        /* ����̨�巢�����̰��yaw�������ֵ */
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

