#include "gimbal_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usart1.h"
#include "can_receive.h"
#include "chassis_task.h"
#include "pid.h"
#include "shoot.h"

/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
  */
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

//��̨���������������
static Gimbal_Control_t gimbal_control ;

extern uint8_t lose_rc;
uint32_t gimbal_high_water;
uint8_t autoshoot_open;

void GIMBAL_task(void *pvParameters)
{
	//�ȴ�������У׼���
	vTaskDelay(3000);
	//��̨��ʼ��
	GIMBAL_Init(&gimbal_control);
	while(1)
	{
		//��̨���ݸ���
		GIMBAL_Feedback_Update(&gimbal_control);
		//��̨����������
		GIMBAL_set_contorl(&gimbal_control);
		//��̨����PID����
		GIMBAL_control_loop(&gimbal_control);
		//printf("%f\r\n",gimbal_control.auto_shoot_point->auto_pit);
		//Ni_Ming(0xf1, gimbal_control.gimbal_yaw_motor.gyro_angle_set, gimbal_control.gimbal_yaw_motor.gyro_angle, 0, 0);
		CAN_CMD_GIMBAL(gimbal_control.gimbal_yaw_motor.given_current, gimbal_control.gimbal_pitch_motor.given_current, 0, 0);
		//CAN_CMD_GIMBAL(0, 0, 0, 0);
		vTaskDelay(1);
		gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
	//��ʼ��ң�ش���
	USART2_Init();
	//�������ָ���ȡ
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//����������ָ���ȡ
	gimbal_init->gimbal_gyro_point = get_GYRO_Measure_Point();  
  //ң��������ָ���ȡ
	gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//��������ָ���ȡ
	gimbal_init->auto_shoot_point = get_auto_shoot_point();
	//��̨pit���PID��ʼ�� 
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, 5000, 0, 12, 0, 0);//kp_out ki_out kp ki kd 20 60
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_acc_pid, 30000, 0, 70, 0, 0);
	//��̨yaw���PID��ʼ��
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, 1500, 0, 20, 0, 0);//kp_out ki_out kp ki kd
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_acc_pid, 4000, 0, 60, 0, 0);
	
	//��̨pit���б�³�ʼ��
	ramp_init(&gimbal_init->gimbal_pitch_motor.ramp, 0.001, 1, -1);
	//��̨yaw���б�³�ʼ��
	ramp_init(&gimbal_init->gimbal_yaw_motor.ramp, 0.001, 1, -1);
	
	kalman_init(&gimbal_init->gimbal_yaw_motor.kalman_t);
	gimbal_init->gimbal_yaw_motor.start_angle = gimbal_init->gimbal_gyro_point->yaw;
}

void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    gimbal_feedback_update->gimbal_pitch_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->pit;//pit�����ǽǶ�
    gimbal_feedback_update->gimbal_pitch_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_x;//pit���ٶ�

    gimbal_feedback_update->gimbal_yaw_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->yaw - gimbal_feedback_update->gimbal_yaw_motor.start_angle;//yaw�����ǽǶ�
    gimbal_feedback_update->gimbal_yaw_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_z;//yaw���ٶ�
}

static uint8_t pir_turn = 0;
//��̨����ǰϵͳʱ��
static uint32_t gimbal_system_time = 0;
 static uint32_t find_enery = 0;
void GIMBAL_set_contorl(Gimbal_Control_t *gimbal_set_control)
{
	static uint32_t send_info = 0;
	static int16_t yaw_channel = 0, pitch_channel = 0;
	//ģʽ�л�λ 0Ϊң��ģʽ��1Ϊ�Զ�ģʽ
	static uint8_t mode_change = 0;
	//��ң���������ݴ������� int16_t yaw_channel,pitch_channel
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);
	
	//��ȡ��ǰϵͳʱ��
	gimbal_system_time = xTaskGetTickCount();
	
	send_info++;
	if(send_info%10 == 0)
	{
		//USART6_Transmit(7000,-gimbal_set_control->gimbal_yaw_motor.gyro_angle * 100);
//		Ni_Ming(0xf1,gimbal_set_control->auto_shoot_point->auto_yaw, gimbal_set_control->gimbal_yaw_motor.gyro_angle_set,\
		gimbal_set_control->gimbal_yaw_motor.gyro_angle+gimbal_set_control->auto_shoot_point->auto_yaw, 0);
	}
	
	//ң���ұ߲��˲�������Ϊ�Զ�ģʽ
	if(gimbal_set_control->gimbal_rc_ctrl->rc.s[0] == 3)
	{
		if(gimbal_set_control->auto_shoot_point->find == 1)
		{
			autoshoot_open = 1;
			find_enery = 0;
			gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = kalman_run(&gimbal_set_control->gimbal_yaw_motor.kalman_t, \
																																				gimbal_set_control->gimbal_yaw_motor.gyro_angle, \
																																				gimbal_set_control->auto_shoot_point->auto_yaw);
			//Ni_Ming(0xf1,gimbal_set_control->gimbal_yaw_motor.gyro_angle_set, gimbal_set_control->gimbal_yaw_motor.gyro_angle, 0, 0);
			//gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = gimbal_set_control->gimbal_yaw_motor.gyro_angle + gimbal_set_control->auto_shoot_point->auto_yaw;
			gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = gimbal_set_control->gimbal_pitch_motor.gyro_angle - gimbal_set_control->auto_shoot_point->auto_pit;
		}	
		else
		{
			find_enery++;
			autoshoot_open = 0;
			if(find_enery > 1000)
			{
				if(gimbal_set_control->gimbal_rc_ctrl->rc.s[1] == 1)
				{
					gimbal_set_control->gimbal_yaw_motor.gyro_angle_set += 0.26f;	
					if(pir_turn == 0)
					{
						gimbal_set_control->gimbal_pitch_motor.gyro_angle_set -= 0.18f;
						if(gimbal_set_control->gimbal_pitch_motor.gyro_angle_set < -28.9f)
						{
							pir_turn = 1;
						}
					}
					if(pir_turn == 1)
					{
						gimbal_set_control->gimbal_pitch_motor.gyro_angle_set += 0.18f;
						if(gimbal_set_control->gimbal_pitch_motor.gyro_angle_set > -0.1f)
						{
							pir_turn = 0;
						}
					}
				}
				if(gimbal_set_control->gimbal_rc_ctrl->rc.s[1] == 3)
				{
					gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = gimbal_set_control->gimbal_yaw_motor.gyro_angle_set;
					gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = gimbal_set_control->gimbal_pitch_motor.gyro_angle_set;
				}
			}
			else
			{
				gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = gimbal_set_control->gimbal_yaw_motor.gyro_angle_set;
				gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = gimbal_set_control->gimbal_pitch_motor.gyro_angle_set;
			}
		}
		mode_change = 1;
		//ң��������0
		yaw_channel = 0;
		pitch_channel = 0;
	}
	
	//ң���ұ߲��˲����м������Ϊң��ģʽ
	else
	{
		if(mode_change == 1)
		{
			//���л���ң��ģʽʱ����̨������ڵ�ǰ������ֵ
			gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = gimbal_set_control->gimbal_yaw_motor.gyro_angle;
			gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = gimbal_set_control->gimbal_pitch_motor.gyro_angle;
			mode_change = 0;
		}
		gimbal_set_control->gimbal_yaw_motor.gyro_angle_set += yaw_channel * Yaw_RC_SEN;
		gimbal_set_control->gimbal_pitch_motor.gyro_angle_set -= pitch_channel * Pitch_RC_SEN;
	}
	
	//�����̨ϵͳ��ǰʱ���ȥ����ң���жϵ�ǰʱ�䣬˵��û���յ�ң���ź�
	if((gimbal_system_time - gimbal_set_control->gimbal_rc_ctrl->time) > 88)
	{
		lose_rc = 1;
		gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = 0;
		gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = gimbal_set_control->gimbal_yaw_motor.gyro_angle;
		gimbal_set_control->gimbal_pitch_motor.gimbal_motor_acc_pid.max_out = 0;
		ramp_init(&gimbal_set_control->gimbal_pitch_motor.ramp, 0.001, 1, -1);
		//gimbal_set_control->gimbal_yaw_motor.start_angle = gimbal_set_control->gimbal_gyro_point->yaw;
	}
	else
	{
		gimbal_set_control->gimbal_pitch_motor.gimbal_motor_acc_pid.max_out = 30000;
		lose_rc = 0;
	}
	
	//��̨pit�Ƕ������޷� 0�� ~ -33��
	if(gimbal_set_control->gimbal_pitch_motor.gyro_angle_set > 16.0f)
	{
		gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = 16.0f;
	}
		if(gimbal_set_control->gimbal_pitch_motor.gyro_angle_set < -35.0f)
	{
		gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = -35.0f;
	}
}

void GIMBAL_control_loop(Gimbal_Control_t *gimbal_control_loop)
{
//	static float pkp,pki,pkd,skp,ski,skd = 0;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp = pkp;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki = pki;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd = pkd;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid.Kp = skp;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid.Ki = ski;
//	gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid.Kd = skd;
	//pit����Ƕ�PID����
//	Ni_Ming(0xf1,gimbal_control_loop->gimbal_yaw_motor.gyro_angle_set, gimbal_control_loop->gimbal_yaw_motor.gyro_angle, gimbal_control_loop->gimbal_yaw_motor.given_current, 0);
	//б�º������1���ڼ���б�º���
	if(gimbal_control_loop->gimbal_pitch_motor.ramp.out < 1)
	{
		ramp_calc(&gimbal_control_loop->gimbal_pitch_motor.ramp, 1);
	}
	if(gimbal_control_loop->gimbal_yaw_motor.ramp.out < 1)
	{
		ramp_calc(&gimbal_control_loop->gimbal_yaw_motor.ramp, 1);
	}
	
	//pit����Ƕ�PID����
	gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set = -PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid, \
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_angle * gimbal_control_loop->gimbal_pitch_motor.ramp.out,\
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_angle_set);
	//yaw����Ƕ�PID����
	gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set = -PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid, \
																																 gimbal_control_loop->gimbal_yaw_motor.gyro_angle, \
																																 gimbal_control_loop->gimbal_yaw_motor.gyro_angle_set);

	//Ni_Ming(0xf1,0, gimbal_control_loop->gimbal_pitch_motor.gyro_angle, 0, 0);
	
	//pit������ٶ�PID����
	gimbal_control_loop->gimbal_pitch_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_acc_pid, \
	                                                                  gimbal_control_loop->gimbal_pitch_motor.gyro_acc, \
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set);
	//yaw������ٶ�PID����
	gimbal_control_loop->gimbal_yaw_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid, \
																																	-gimbal_control_loop->gimbal_yaw_motor.gyro_acc, \
																																	gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set);
}

//��̨���PID��ʼ��
void GIMBAL_PID_Init(PidTypeDef *pid, float maxout, float max_iout, float kp, float ki, float kd)
{
	if (pid == NULL)
	{
			return;
	}
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	pid->set = 0.0f;

	pid->max_iout = max_iout;
	pid->max_out = maxout;
}

