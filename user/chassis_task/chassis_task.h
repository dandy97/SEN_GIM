#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "rc.h"

#include "can_receive.h"
#include "pid.h"
#include "user_lib.h"
#include "auto_shoot.h"


//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 4
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.004

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//ң������������
#define CHASSIS_RC_DEADLINE 10
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//���̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
//  const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
//  const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
//  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
//  chassis_mode_e chassis_mode;               //���̿���״̬��
//  chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
//  PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;
	ramp_function_source_t ramp_add;
	ramp_function_source_t ramp_reduce;

  float vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
//  fp32 chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad/s
//  fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
//  fp32 chassis_yaw_set;

  float vx_max_speed;  //ǰ����������ٶ� ��λm/s
  float vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  float vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  float vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
	float speed_send;    //�����ٶ�         ��λm/s
//  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
//  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
//  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
} chassis_move_t;

extern chassis_move_t chassis_move;

//��������
void chassis_task(void *pvParameters);
//���̳�ʼ������Ҫ��pid��ʼ��
void chassis_init(chassis_move_t *chassis_move_init);
//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_move_update);
//���̿���������
void chassis_set_contorl(chassis_move_t *chassis_move_control);
//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(float *vx_set, chassis_move_t *chassis_move_rc_to_vector);
//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

//���͸����ص�����̼ƺ���̨�Ƕ�
void USART6_Transmit(int16_t chassis_dis, int16_t gimabl_angle);
#endif
