#ifndef LED_H
#define LED_H

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;

void LED_Init(void);
void GREEN_LED(uint16_t bright);
void RED_LED(uint16_t bright);

void FRIC_Init(void);
#endif
