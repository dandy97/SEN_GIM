#ifndef SHOOT_H
#define SHOOT_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "can_receive.h"

typedef unsigned char bool_t;//0��1

//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f

//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

typedef struct
{
	const motor_measure_t *shoot_motor_measure;
	float speed;
	float speed_set;
	float angle;
	float set_angle;
	int16_t given_current;
	int8_t ecd_count;

	bool_t press_l;
	bool_t press_r;
	bool_t last_press_l;
	bool_t last_press_r;
	uint16_t press_l_time;
	uint16_t press_r_time;
	uint16_t rc_s_time;

	bool_t move_flag;
	uint32_t cmd_time;
	uint32_t run_time;
	bool_t key;
	uint16_t key_time;
	bool_t shoot_done;
	uint8_t shoot_done_time;
	int16_t BulletShootCnt;
	int16_t last_butter_count;
} Shoot_Motor_t;

//�����ʼ��
extern void shoot_init(void);
//������ݸ���
void Shoot_Feedback_Update(void);
//���ѭ��
int16_t shoot_control_loop(void);

#endif
