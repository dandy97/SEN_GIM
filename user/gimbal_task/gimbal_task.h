#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stm32f4xx_hal.h" 

#include "can_receive.h"
#include "rc.h"
#include "auto_shoot.h"
#include "user_lib.h"
#include "pid.h"
#include "kalman_filter.h"

//yaw,pitch����ͨ���Լ�״̬����ͨ��
#define YawChannel 2
#define PitchChannel 3
//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_deadband 10
//yaw��pitch�Ƕ���ң�����������
#define Yaw_RC_SEN -0.0006f
#define Pitch_RC_SEN -0.0005f //0.005

//pitch �ǶȻ� 
#define PITCH_GYRO_PID_KP 0.0f
#define PITCH_GYRO_PID_KI 0.0f
#define PITCH_GYRO_PID_KD 0.0f

#define PITCH_GYRO_PID_MAX_OUT 5000.0f
#define PITCH_GYRO_PID_MAX_IOUT 0.0f

//pitch ���ٶȻ� 
#define PITCH_ACC_PID_KP 0.0f
#define PITCH_ACC_PID_KI 0.0f
#define PITCH_ACC_PID_KD 0.0f

#define PITCH_ACC_PID_MAX_OUT 30000.0f
#define PITCH_ACC_PID_MAX_IOUT 0.0f

typedef struct
{
	float kp;
	float ki;
	float kd;

	float set;
	float get;
	float err;

	float max_out;
	float max_iout;

	float Pout;
	float Iout;
	float Dout;

	float out;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
		
		kalman kalman_t;
		ramp_function_source_t ramp;
//    Gimbal_PID_t gimbal_motor_absolute_angle_pid;
//    Gimbal_PID_t gimbal_motor_relative_angle_pid;
    PidTypeDef gimbal_motor_gyro_pid;
	  PidTypeDef gimbal_motor_acc_pid;
//    gimbal_motor_mode_e gimbal_motor_mode;
//    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
		uint16_t last_ecd;
		float rount_cnt;
    float max_relative_angle; //rad
    float min_relative_angle; //rad

		float pit_mode_change_angle;
		float yaw_mode_change_angle;
    float relative_angle;     //rad
    float relative_angle_set; //rad
    float gyro_angle;     //rad
    double gyro_angle_set; //rad
    float gyro_acc;         //rad/s
    float gyro_acc_set;
    float motor_speed;
    float raw_cmd_current;
    float current_set;
		float start_angle;
    int16_t given_current;

} Gimbal_Motor_t;

typedef struct
{
    float max_yaw;
    float min_yaw;
    float max_pitch;
    float min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const gyro_info_t *gimbal_gyro_point;
	  const auto_shoot_data_t *auto_shoot_point;
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
    Gimbal_Cali_t gimbal_cali;
} Gimbal_Control_t;

//��̨����
void GIMBAL_task(void *pvParameters); 
//��ʼ����̨
void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
//��̨���ݸ���
void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//������̨��������ֵ����ԽǶ�
float motor_ecd_to_angle_change(uint16_t ecd, uint16_t last_ecd, float round_cnt);
//��̨����������
void GIMBAL_set_contorl(Gimbal_Control_t *chassis_move_control);
//��̨����pid����
void GIMBAL_control_loop(Gimbal_Control_t *gimbal_control_loop);
//��̨���PID��ʼ��
void GIMBAL_PID_Init(PidTypeDef *pid, float maxout, float max_iout, float kp, float ki, float kd);
#endif
