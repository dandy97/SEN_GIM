#include "chassis_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

#include "usart1.h"
#include "rc.h"
#include "auto_shoot.h"

#include "can_receive.h"
#include "pid.h"
#include "user_lib.h"
#include "math.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//�������ݽṹ��
chassis_move_t chassis_move;
int32_t chassis_dis;
static int16_t shoot_speed_send = 0;
extern uint8_t autoshoot_open;
uint8_t lose_rc;
uint8_t fric_power;
static uint8_t open_fric = 0;
static uint32_t fric_time = 0;
//��������ռ�ʣ����
uint32_t chassis_high_water;
void chassis_task(void *pvParameters)
{
	//����һ��ʱ��
  vTaskDelay(8000);
	open_fric = 0;
	//���̳�ʼ��
	chassis_init(&chassis_move);
	while(1)
	{
		//���̿���������
		chassis_set_contorl(&chassis_move);
		//�����ٶȷ���
		if(fric_power == 3)
		{
			open_fric = 1;
			fric_time = 0;
		}
		else
		{
			fric_time++;
			if(fric_time > 2000)
			{
				open_fric = 0;
			}
		}
		
		if(open_fric == 1)
		{
			shoot_speed_send = 1000;
		}
		else if(chassis_move.chassis_RC->rc.s[0] == 3)
		{
			ramp_init(&chassis_move.ramp_reduce, 0.4, 140, 0);
			ramp_calc(&chassis_move.ramp_add, 1);			
			shoot_speed_send = 1000 + chassis_move.ramp_add.out;
		}
		else if(chassis_move.chassis_RC->rc.s[1] == 1)
		{
			ramp_init(&chassis_move.ramp_add, 0.4, 140, 0);
			ramp_calc(&chassis_move.ramp_reduce, 1);
			shoot_speed_send = 1140 -  chassis_move.ramp_reduce.out;
		}
		else if(chassis_move.chassis_RC->rc.s[1] == 3)
		{
			ramp_init(&chassis_move.ramp_reduce, 0.4, 140, 0);
			ramp_calc(&chassis_move.ramp_add, 1);			
			shoot_speed_send = 1000 + chassis_move.ramp_add.out;
		}
		else if(chassis_move.chassis_RC->rc.s[1] == 2)
		{
			ramp_init(&chassis_move.ramp_reduce, 0.4, 140, 0);
			ramp_calc(&chassis_move.ramp_add, 1);			
			shoot_speed_send = 1000 + chassis_move.ramp_add.out;
		}
		else
		{
			ramp_init(&chassis_move.ramp_add, 0.4, 140, 0);
			ramp_init(&chassis_move.ramp_add, 0.4, 140, 0);
			shoot_speed_send = 1000;
		}
		TIM4->CCR1 = TIM4->CCR2 = shoot_speed_send;
 		CAN_CMD_CHASSIS(shoot_speed_send, chassis_move.speed_send, chassis_move.chassis_RC->rc.s[1], chassis_move.chassis_RC->rc.s[0], autoshoot_open);
		//printf("%f\r\n",chassis_move.speed_send);
		vTaskDelay(pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//���̳�ʼ��
void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}
	//�����ٶ�һ���˲���ʼ��
	const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
		
	//��ȡң����ָ��
  chassis_move_init->chassis_RC = get_remote_control_point();
	
	//Ħ����б�³�ʼ��
	ramp_init(&chassis_move_init->ramp_add, 0.4, 250, 0);
	ramp_init(&chassis_move_init->ramp_reduce, 0.4, 250, 0);
	
 //��һ���˲�����б����������
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	
	autoshoot_open = 0;
}

//��������ǰϵͳʱ��
static uint32_t chassis_system_time = 0;
//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(float *vx_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL)
	{
		return;
	}
	//ң����ԭʼͨ��ֵ
	int16_t vx_channel;
	float vx_set_channel;
	//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
	rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);

	vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
	
	//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);

	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}
	
	//��ȡ��ǰϵͳʱ��
	chassis_system_time = xTaskGetTickCount();
	
	//�������ϵͳ��ǰʱ���ȥ����ң���жϵ�ǰʱ�䣬˵��û���յ�ң���ź�
	if((chassis_system_time - chassis_move_rc_to_vector->chassis_RC->time) > 88)
	{
		*vx_set = 0;
	}
	else
	{
		*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	}
	
}

//����ң�������������
void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

	if (chassis_move_control == NULL)
	{
			return;
	}

	//�����ٶ�
	float vx_set = 0.0f;
	//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
	chassis_rc_to_control_vector(&vx_set, chassis_move_control);
	
	chassis_move_control->speed_send = vx_set;
}

/*����������*/
//���͸����ص�����̼ƺ���̨�Ƕ�
void USART6_Transmit(int16_t chassis_dis, int16_t gimabl_angle)
{
	static uint8_t SEND_DATA[9]={0};
//	SEND_DATA[0] = 0xaa;
//	SEND_DATA[1] = 0;
//	SEND_DATA[2] = 0;
//	SEND_DATA[3] = (uint8_t)(gimabl_angle >> 8);//yaw
//	SEND_DATA[4] = (uint8_t)gimabl_angle;
//	SEND_DATA[5] = (uint8_t)(chassis_dis >> 8);
//	SEND_DATA[6] = (uint8_t)chassis_dis;
//	SEND_DATA[7] = 0;
//	SEND_DATA[8] = 0xbb ;
	
	SEND_DATA[0] = 0xaa;
	SEND_DATA[1] = 0;
	SEND_DATA[2] = 1;
	SEND_DATA[3] = 2;//yaw
	SEND_DATA[4] = 3;
	SEND_DATA[5] = 4;
	SEND_DATA[6] = 5;
	SEND_DATA[7] = 6;
	SEND_DATA[8] = 0xbb ;

	HAL_UART_Transmit_DMA(&huart6,SEND_DATA,9);
}
