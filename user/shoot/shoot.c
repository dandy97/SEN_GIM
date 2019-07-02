#include "shoot.h"
#include "stm32f4xx_hal.h"

#include "rc.h"
#include "can_receive.h"
#include "pid.h"
static Shoot_Motor_t trigger_motor;          //射击数据

static const RC_ctrl_t *shoot_rc; //遥控器指针
static PidTypeDef trigger_motor_pid;         //电机PID

void shoot_init(void)
{
	static const float Trigger_speed_pid[3] = {0, 0, 0};//P I D
	//遥控器指针
	shoot_rc = get_remote_control_point();
	//电机指针
	trigger_motor.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
	//初始化PID                                                   PID_MAX_OUT     PID_MAX_IOUT
	PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, 10000,          1000);
	//更新数据
	Shoot_Feedback_Update();

}

//射击数据更新
void Shoot_Feedback_Update(void)
{
	static float speed_fliter_1 = 0.0f;
	static float speed_fliter_2 = 0.0f;
	static float speed_fliter_3 = 0.0f;

	//拨弹电机速度滤波
	static const float fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	//二阶低通滤波
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
	trigger_motor.speed = speed_fliter_3;

	//射击开关下档时间计时
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

//射击循环
int16_t shoot_control_loop(void)
{
	int16_t shoot_CAN_Set_Current; //返回的can值

	Shoot_Feedback_Update(); //更新数据
	
	//计算拨弹轮电机PID
	PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
	
	trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
	shoot_CAN_Set_Current = trigger_motor.given_current;
}
