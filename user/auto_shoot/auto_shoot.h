#ifndef AUTO_SHOOT_H
#define AUTO_SHOOT_H

#include "stm32f4xx_hal.h"

typedef struct
{
	uint8_t find;
	float   auto_pit;
	float   auto_yaw;
	uint8_t cat_walk;
	uint8_t go_point;
} auto_shoot_data_t;

extern UART_HandleTypeDef huart6;

extern DMA_HandleTypeDef  UART6RxDMA_Handler;

//��ʼ�����ش���
void Auto_Shoot_Init(void);
//����������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
extern const auto_shoot_data_t *get_auto_shoot_point(void);

#endif
