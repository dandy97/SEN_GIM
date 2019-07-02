#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "stm32f4xx_hal.h"
#include "can.h"


//电机can接收ID
typedef enum
{
	CAN_YAW_MOTOR_ID = 0x205,
	CAN_PIT_MOTOR_ID = 0x206,
	CAN_TRIGGER_MOTOR_ID = 0x207,
	CAN_GIMBAL_ALL_ID = 0x1FF,
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
	uint16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_ecd;
	int16_t offset_ecd;
	float round_cnt;
	float ecd_raw_rate;
	float total_ecd;
	float angle;
} motor_measure_t;

//陀螺仪数据结构体
typedef struct
{
	float v_x;
	float v_z;
	float pit;
	float yaw;
} gyro_info_t;

//统一处理can中断函数
void CAN_hook(CAN_RxHeaderTypeDef *rx_message, uint8_t *Data);
//发送底盘电机、摩擦轮电机、拨盘电机控制命令
void CAN_CMD_CHASSIS(int16_t shoot_speed, float chassis_speed, uint8_t shoot, uint8_t control, uint8_t auto_shoot);
//发送陀螺仪控制命令
void CAN_CMD_GYRO_CALI(uint8_t mode, uint16_t time);
//发送云台电机控制命令
void CAN_CMD_GIMBAL(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//云台数据获取
void get_gimbal_motor_measuer(motor_measure_t* ptr, uint8_t* Data);
//发送摩擦轮电机控制命令
void CAN_CMD_SHOOT(int16_t shoot_speed);
//返回陀螺仪变量地址，通过指针方式获取原始数据
extern const gyro_info_t *get_GYRO_Measure_Point(void);
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回拨弹电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
#endif
