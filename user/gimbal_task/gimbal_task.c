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
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
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

//云台控制所有相关数据
static Gimbal_Control_t gimbal_control ;

extern uint8_t lose_rc;
uint32_t gimbal_high_water;
uint8_t autoshoot_open;

void GIMBAL_task(void *pvParameters)
{
	//等待陀螺仪校准完毕
	vTaskDelay(3000);
	//云台初始化
	GIMBAL_Init(&gimbal_control);
	while(1)
	{
		//云台数据更新
		GIMBAL_Feedback_Update(&gimbal_control);
		//云台控制量设置
		GIMBAL_set_contorl(&gimbal_control);
		//云台控制PID计算
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
	//初始化遥控串口
	USART2_Init();
	//电机数据指针获取
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
	//陀螺仪数据指针获取
	gimbal_init->gimbal_gyro_point = get_GYRO_Measure_Point();  
  //遥控器数据指针获取
	gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
	//自瞄数据指针获取
	gimbal_init->auto_shoot_point = get_auto_shoot_point();
	//云台pit电机PID初始化 
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, 5000, 0, 12, 0, 0);//kp_out ki_out kp ki kd 20 60
	GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_acc_pid, 30000, 0, 70, 0, 0);
	//云台yaw电机PID初始化
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, 1500, 0, 20, 0, 0);//kp_out ki_out kp ki kd
	GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_acc_pid, 4000, 0, 60, 0, 0);
	
	//云台pit电机斜坡初始化
	ramp_init(&gimbal_init->gimbal_pitch_motor.ramp, 0.001, 1, -1);
	//云台yaw电机斜坡初始化
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
    //云台数据更新
    gimbal_feedback_update->gimbal_pitch_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->pit;//pit陀螺仪角度
    gimbal_feedback_update->gimbal_pitch_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_x;//pit加速度

    gimbal_feedback_update->gimbal_yaw_motor.gyro_angle = gimbal_feedback_update->gimbal_gyro_point->yaw - gimbal_feedback_update->gimbal_yaw_motor.start_angle;//yaw陀螺仪角度
    gimbal_feedback_update->gimbal_yaw_motor.gyro_acc = gimbal_feedback_update->gimbal_gyro_point->v_z;//yaw加速度
}

static uint8_t pir_turn = 0;
//云台任务当前系统时间
static uint32_t gimbal_system_time = 0;
 static uint32_t find_enery = 0;
void GIMBAL_set_contorl(Gimbal_Control_t *gimbal_set_control)
{
	static uint32_t send_info = 0;
	static int16_t yaw_channel = 0, pitch_channel = 0;
	//模式切换位 0为遥控模式，1为自动模式
	static uint8_t mode_change = 0;
	//将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
	rc_deadline_limit(gimbal_set_control->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);
	
	//获取当前系统时间
	gimbal_system_time = xTaskGetTickCount();
	
	send_info++;
	if(send_info%10 == 0)
	{
		//USART6_Transmit(7000,-gimbal_set_control->gimbal_yaw_motor.gyro_angle * 100);
//		Ni_Ming(0xf1,gimbal_set_control->auto_shoot_point->auto_yaw, gimbal_set_control->gimbal_yaw_motor.gyro_angle_set,\
		gimbal_set_control->gimbal_yaw_motor.gyro_angle+gimbal_set_control->auto_shoot_point->auto_yaw, 0);
	}
	
	//遥控右边拨杆拨到最上为自动模式
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
		//遥控输入清0
		yaw_channel = 0;
		pitch_channel = 0;
	}
	
	//遥控右边拨杆拨到中间和最下为遥控模式
	else
	{
		if(mode_change == 1)
		{
			//当切换到遥控模式时，云台输入等于当前陀螺仪值
			gimbal_set_control->gimbal_yaw_motor.gyro_angle_set = gimbal_set_control->gimbal_yaw_motor.gyro_angle;
			gimbal_set_control->gimbal_pitch_motor.gyro_angle_set = gimbal_set_control->gimbal_pitch_motor.gyro_angle;
			mode_change = 0;
		}
		gimbal_set_control->gimbal_yaw_motor.gyro_angle_set += yaw_channel * Yaw_RC_SEN;
		gimbal_set_control->gimbal_pitch_motor.gyro_angle_set -= pitch_channel * Pitch_RC_SEN;
	}
	
	//如果云台系统当前时间减去进入遥控中断当前时间，说明没有收到遥控信号
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
	
	//云台pit角度输入限幅 0° ~ -33°
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
	//pit电机角度PID计算
//	Ni_Ming(0xf1,gimbal_control_loop->gimbal_yaw_motor.gyro_angle_set, gimbal_control_loop->gimbal_yaw_motor.gyro_angle, gimbal_control_loop->gimbal_yaw_motor.given_current, 0);
	//斜坡函数输出1后不在计算斜坡函数
	if(gimbal_control_loop->gimbal_pitch_motor.ramp.out < 1)
	{
		ramp_calc(&gimbal_control_loop->gimbal_pitch_motor.ramp, 1);
	}
	if(gimbal_control_loop->gimbal_yaw_motor.ramp.out < 1)
	{
		ramp_calc(&gimbal_control_loop->gimbal_yaw_motor.ramp, 1);
	}
	
	//pit电机角度PID计算
	gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set = -PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_gyro_pid, \
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_angle * gimbal_control_loop->gimbal_pitch_motor.ramp.out,\
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_angle_set);
	//yaw电机角度PID计算
	gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set = -PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_gyro_pid, \
																																 gimbal_control_loop->gimbal_yaw_motor.gyro_angle, \
																																 gimbal_control_loop->gimbal_yaw_motor.gyro_angle_set);

	//Ni_Ming(0xf1,0, gimbal_control_loop->gimbal_pitch_motor.gyro_angle, 0, 0);
	
	//pit电机角速度PID计算
	gimbal_control_loop->gimbal_pitch_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_acc_pid, \
	                                                                  gimbal_control_loop->gimbal_pitch_motor.gyro_acc, \
																																		gimbal_control_loop->gimbal_pitch_motor.gyro_acc_set);
	//yaw电机角速度PID计算
	gimbal_control_loop->gimbal_yaw_motor.given_current = PID_Calc(&gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_acc_pid, \
																																	-gimbal_control_loop->gimbal_yaw_motor.gyro_acc, \
																																	gimbal_control_loop->gimbal_yaw_motor.gyro_acc_set);
}

//云台电机PID初始化
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

