/**
 *****************************��ݸ����ѧԺACEʵ���� *****************************
 * @file 			can_1_receive.c/h
 *
 * @brief 	�˲�����ʼ���Լ�������̨��can1����
 *
 * @note  		
 * @history
 *
 @verbatim
 ==============================================================================


==============================================================================
 @endverbatim
 *****************************��ݸ����ѧԺACEʵ���� *****************************
 */


#include "CAN_1_Receive.h"
#include "main.h"
#include "SysInit.h"

extern FDCAN_HandleTypeDef hfdcan1;                     // ���
extern FDCAN_HandleTypeDef hfdcan2;

//void (*CAN1_receive_callback)(FDCAN_HandleTypeDef *);


/*--------------------����-----------------------*/

#if (gimbal_yaw_TO_chassis == 1)
//����yaw��3508�������
static motor_measure_t motor_yaw;

//���ظ�yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
motor_measure_t *Chassis_Get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}

#endif

//�������̵������ static
motor_measure_t motor_chassis[4];
//���������������
static motor_measure_t motor_fire_A;
static motor_measure_t motor_fire_B;
//����pitch��������
 motor_measure_t motor_pitch;

//������ݶ�ȡ
#define get_motor_M3508(ptr, rx_message)                                                  \
    {                                                                                     \
        (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]); \
        (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);    \
    }


//���ز������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *Get_Fire_MotorA_Measure_Point(void)
{
    return &motor_fire_A;
}
const motor_measure_t *Get_Fire_MotorB_Measure_Point(void)
{
    return &motor_fire_B;
}
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];  //(i & 0x03)
}

//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ���� 
motor_measure_t *Get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pitch;
}


/**
	* @brief		�˲�������
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
  
 
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE); // ����ȫ�ֹ������Ծܾ����в�ƥ���֡ 
	

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);  // ������FDCANʵ���ϼ���Rx FIFO 0����Ϣ֪ͨ 

	
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);			// ��������ģʽ 
	HAL_FDCAN_Start(&hfdcan1);																														// ����FDCAN

}


/**
	* @brief		HAl��can1�Ļص�����
	* @param		��������� CAN�ľ��
	* @retval   none
  */

void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
/*     CAN_RxHeaderTypeDef	Rxmessage;	
 */    
    if(hcan == &hfdcan1)
				CAN1_chassis_receive(hcan);
		if(hcan == &hfdcan2)
				CAN2_chassis_receive(hcan);
//			if(CAN1_receive_callback != NULL)
//					CAN1_receive_callback(hcan)	;               
		
	

}


/*************************************can1���մ�������*************************************/

 void CAN1_chassis_receive(FDCAN_HandleTypeDef *hcan)
{
  	FDCAN_RxHeaderTypeDef	rx_message;															//������Ϣ�ṹ��

   	uint8_t Rx_Data[8];																					//���յ���Ϣ���������
		
  if( HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_message, Rx_Data) == HAL_OK)	//��ȡ���յ���Ϣ
	{	
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		
    switch (rx_message.Identifier)
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
				
      #if (pitch_angle_position == 1)

        /* ������ */
				case 0x01:
				{
					
					motor_pitch.position = ((uint16_t)(Rx_Data[6] << 24) | (uint16_t)(Rx_Data[5] << 16) | (uint16_t)(Rx_Data[4] << 8) | (uint16_t)(Rx_Data[3] ));
					
				if(motor_pitch.position >= 0 && motor_pitch.position<=150)
					{
						motor_pitch.position +=884;
					}
				else
						motor_pitch.position -= 140;
            motor_pitch.actual_Position = motor_pitch.position;   //ʵ��ֵ��ȥ�м�ֵ
            motor_pitch.pitch_angle = -(motor_pitch.actual_Position * 360 / 1024 / PITCH_GR - Pitch_Middle_Angle);
            break;
				}
      #endif
					/* pitch���� */
        case CAN_PIT_MOTOR_ID:
        {

            motor_pitch.speed = (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

      #if (pitch_angle_position == 0)
          	motor_pitch.position  = (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
						Motor_Actual_Position (&motor_pitch,1.5*19,8192);
						motor_pitch.pitch_angle  =( (motor_pitch.actual_Position*360/(8192)/1.5/19) - 47);

      #endif 

          break;
        }

      /* ���̽���can1��̨��������� */
      #if (gimbal_yaw_TO_chassis == 1)
        case 0x208:
        {
          	motor_yaw.position 					= (uint16_t)((Rx_Data[0] << 8) + Rx_Data[1]);
            Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //����yaw�������ʵ����ֵ
            motor_yaw.yaw_angle				  = motor_yaw.actual_Position * 360 / 8192 / YAW_RATIO;
            motor_yaw.speed 				 		= (uint16_t)((Rx_Data[2] << 8) + Rx_Data[3]);

          break;
        }

       #endif 
      /* �������A */
        case CAN_TRIGGER_MOTORA_ID:
        {
          motor_fire_A.position =  (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
          Motor_Actual_Position(&motor_fire_A, Sec_YAW_RATIO , 8192);
          motor_fire_A.speed   =   (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

          break;
        }

      /* �������A */
        case CAN_TRIGGER_MOTORB_ID:
        {
          motor_fire_B.position =  (uint16_t)((Rx_Data[0] << 8) | (Rx_Data[1]));
          Motor_Actual_Position(&motor_fire_B, Sec_YAW_RATIO , 8192);
          motor_fire_B.speed   =   (uint16_t)((Rx_Data[2] << 8) | (Rx_Data[3]));

          break;
        }

			 default:
        {
            break;
        }
		}
	}


}