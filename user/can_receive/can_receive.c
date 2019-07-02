#include "can_receive.h"
#include "usart1.h"

//����������ݶ�ȡ

extern int32_t chassis_dis;
//�����������
static motor_measure_t motor_yaw, motor_pit, motor_trigger;
//�����Ǳ���
static gyro_info_t gyro_info;
//CAN�����жϻص�����
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == (&hcan1))
	{
		uint8_t Data[8];
		CAN_RxHeaderTypeDef RxMeg;
		HAL_StatusTypeDef	HAL_RetVal;
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMeg, Data);
		if(HAL_OK==HAL_RetVal)
		{
			switch (RxMeg.StdId)
			{
				case 0x200:
				{
					chassis_dis = (Data[6]<<8 | Data[7]);
					break;
				}
				case CAN_YAW_MOTOR_ID:
				{
					//���������ݺ꺯��
					get_gimbal_motor_measuer(&motor_yaw, Data);
					break;
				}
				case CAN_PIT_MOTOR_ID:
				{
					//���������ݺ꺯��
					get_gimbal_motor_measuer(&motor_pit, Data);
					break;
				}	
				default:
				{
					break;
				}			
			}
		}
	}
	if(hcan == (&hcan2))
	{
		uint8_t Data[8];
		CAN_RxHeaderTypeDef RxMeg;
		HAL_StatusTypeDef	HAL_RetVal;
		HAL_RetVal = HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxMeg, Data);
		if(HAL_OK==HAL_RetVal)
		{
			switch (RxMeg.StdId)
			{
				case 101:
				{
					static int16_t pitch_connt = 0;
					static int16_t raw_v_x, raw_v_z, raw_pit = 0;
					static float pitch_angle, last_pitch_angle = 0;
					raw_v_x = Data[0]<<8 | Data[1];
					raw_v_z = Data[2]<<8 | Data[3];
					raw_pit = Data[4]<<8 | Data[5];
					
					//������ԭʼ�����ǻ��ȣ��ѻ���ת��Ϊ�Ƕ�
					gyro_info.v_x = (float)raw_v_x * 0.057295f;
					gyro_info.v_z = (float)raw_v_z * 0.057295f;
					
					//������ԭʼ���ݱ�����100��
					pitch_angle = (float)raw_pit/100;
					
					//pit�Ƕ�û�и�ֵ
					if(pitch_angle < 0)
					{
						pitch_angle = pitch_angle + 360;
					}
					
					//���Ƕȸĳ������ģ�����360����0��
					if((pitch_angle - last_pitch_angle) > 330)
						pitch_connt--;
					else if((pitch_angle - last_pitch_angle) < -330)
						pitch_connt++;
					
					gyro_info.pit = pitch_angle + pitch_connt * 360;
					last_pitch_angle = pitch_angle;
					break;
				}
				case 0x401:
				{
					gyro_info.yaw = (float)(0.008571428571f)*((int32_t)(Data[0]<<24)|(int32_t)(Data[1]<<16) | (int32_t)(Data[2]<<8) | (int32_t)(Data[3])); 
					break;
				}
//				case 0x80:
//				{
//					static int16_t raw_yaw;
//					raw_yaw = Data[0]<<8 | Data[1];
//					gyro_info.yaw = (float)raw_yaw/100; 
//					Ni_Ming(0xf1,gyro_info.yaw,0,0,0);
//					break;
//				}
				default:
				{
					break;
				}
			}
		}
	}
}

//��̨���ݻ�ȡ
void get_gimbal_motor_measuer(motor_measure_t* ptr, uint8_t* Data)
{
  ptr->last_ecd = ptr->ecd;
  ptr->ecd      = Data[0] << 8 | Data[1];

	if (ptr->ecd - ptr->last_ecd > 4096)
	{
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
	
	else if (ptr->ecd - ptr->last_ecd < -4096)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  /* total angle, unit is degree */
	ptr->angle = ptr->total_ecd / (8192.0f/360.0f);

}
//���͵��̵����Ħ���ֵ�������̵����������
void CAN_CMD_CHASSIS(int16_t shoot_speed, float chassis_speed, uint8_t shoot, uint8_t control, uint8_t auto_shoot)
{
  static int16_t chassis_speed_send;
	chassis_speed_send = chassis_speed * 100;
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	TxMeg.StdId = 0x300;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x08;
	Data[0] = shoot_speed >> 8;
	Data[1] = shoot_speed;
	Data[2] = chassis_speed_send >> 8;
	Data[3] = chassis_speed_send;;
	Data[4] = shoot;
	Data[5] = 0;
	Data[6] = control;
	Data[7] = auto_shoot;

	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//������̨�����������
void CAN_CMD_GIMBAL(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = CAN_GIMBAL_ALL_ID;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x08;
	Data[0] = motor1 >> 8;
	Data[1] = motor1;
	Data[2] = motor2 >> 8;
	Data[3] = motor2;
	Data[4] = motor3 >> 8;
	Data[5] = motor3;
	Data[6] = motor4 >> 8;
	Data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//����Ħ���ֵ����������
void CAN_CMD_SHOOT(int16_t shoot_speed)//(1000~1500)
{
	uint8_t Data[8];
	uint32_t pTxMailbox;
	CAN_TxHeaderTypeDef TxMeg;
	
	TxMeg.StdId = 0x149;
	TxMeg.IDE = CAN_ID_STD;
	TxMeg.RTR = CAN_RTR_DATA;
	TxMeg.DLC = 0x08;
	Data[0] = shoot_speed >> 8;
	Data[1] = shoot_speed;
	Data[2] = 0;
	Data[3] = 0;
	Data[4] = 0;
	Data[5] = 0;
	Data[6] = 0;
	Data[7] = 0;

	HAL_CAN_AddTxMessage(&hcan1, &TxMeg, Data, &pTxMailbox);
}

//���������ǿ������� �����ǽ���ID��100; mode:0x30ΪУ׼ģʽ��timeΪУ׼ʱ�䣬1000ms���Ҿ��С�
void CAN_CMD_GYRO_CALI(uint8_t mode, uint16_t time)
{
//	uint8_t Data[8];
//	uint32_t pTxMailbox;
//	CAN_TxHeaderTypeDef TxMeg;
//	
//	TxMeg.StdId = 0x404;
//	TxMeg.IDE = CAN_ID_STD;
//	TxMeg.RTR = CAN_RTR_DATA;
//	TxMeg.DLC = 0x08;
//  Data[0] = 0;
//	Data[1] = 1;
//  Data[2] = 2 ;
//	Data[3] = 3;
//	Data[4] = 4;
//	Data[5] = 5;
//	Data[6] = 6;
//	Data[7] = 7;
	
	//10ms��һֱ����У׼ָ���������
	uint32_t tickstart = HAL_GetTick();
  uint32_t wait = 10;

  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)1;
  }

  while((HAL_GetTick() - tickstart) < wait)
  {
	 // HAL_CAN_AddTxMessage(&hcan2, &TxMeg, Data, &pTxMailbox);
  }
}

//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//���ز������������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
	  return &motor_trigger;
}
//���������Ǳ�����ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const gyro_info_t *get_GYRO_Measure_Point(void)
{
	return &gyro_info;
}
