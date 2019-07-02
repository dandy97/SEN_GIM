#include "shoot.h"
#include "stm32f4xx_hal.h"

#include "rc.h"
#include "can_receive.h"
#include "pid.h"
static Shoot_Motor_t trigger_motor;          //�������

static const RC_ctrl_t *shoot_rc; //ң����ָ��
static PidTypeDef trigger_motor_pid;         //���PID

void shoot_init(void)
{
	static const float Trigger_speed_pid[3] = {0, 0, 0};//P I D
	//ң����ָ��
	shoot_rc = get_remote_control_point();
	//���ָ��
	trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
	//��ʼ��PID                                                   PID_MAX_OUT     PID_MAX_IOUT
	PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, 10000,          1000);
	//��������
	Shoot_Feedback_Update();

}

//������ݸ���
void Shoot_Feedback_Update(void)
{
	static float speed_fliter_1 = 0.0f;
	static float speed_fliter_2 = 0.0f;
	static float speed_fliter_3 = 0.0f;

	//��������ٶ��˲�
	static const float fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	//���׵�ͨ�˲�
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
	trigger_motor.speed = speed_fliter_3;

	//��������µ�ʱ���ʱ
	if (switch_is_down(shoot_rc->rc.s[1]))
	{
		if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
		{
			trigger_motor.rc_s_time++;
		}
	}
	else
	{
		trigger_motor.rc_s_time = 0;
	}
}

//���ѭ��
int16_t shoot_control_loop(void)
{
	int16_t shoot_CAN_Set_Current; //���ص�canֵ

	Shoot_Feedback_Update(); //��������
	
	//���㲦���ֵ��PID
	PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
	
	trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
	shoot_CAN_Set_Current = trigger_motor.given_current;
}
