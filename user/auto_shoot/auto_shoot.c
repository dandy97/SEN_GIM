#include "auto_shoot.h"


UART_HandleTypeDef huart6;
static uint8_t Rx_data[3][50];
static auto_shoot_data_t auto_shoot_data;
static uint16_t pc_count= 36;

void Auto_Shoot_Init(void)
{
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;	
  HAL_UART_Init(&huart6);
	
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
	SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart6, (uint8_t *)Rx_data, pc_count);	
}

void USART6_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) &&   /*读取USART_SR寄存器的IDLE位*/
		  __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE))
	{
		uint16_t tmp = huart6.Instance->SR;  //检测到空闲线路，硬件置1，软件清0 （读入USART_DR和USART_SR）
		tmp = huart6.Instance->DR;
		tmp--;
		CLEAR_BIT(huart6.Instance->SR, USART_SR_IDLE); //IDLE位清零
		__HAL_DMA_DISABLE(huart6.hdmarx);
		uint16_t temp = huart6.hdmarx->Instance->NDTR;  //要传输的剩余数据项数
		if((pc_count - temp) == 9)//传输数组长度 - 传输的剩余数据项数
		{
			if((Rx_data[0][0]==0xaa)&&(Rx_data[0][8]==0xbb))
			{
				auto_shoot_data.find = Rx_data[0][5];
				if(Rx_data[0][5] == 1)//是否看见
				{
					auto_shoot_data.auto_pit = (float)((short)(Rx_data[0][2]<<8 | Rx_data[0][1]) *90/ 32767.0f);  //pit
					auto_shoot_data.auto_yaw = (float)((short)(Rx_data[0][4]<<8 | Rx_data[0][3]) *90/ 32767.0f); 	 //yaw	
					//printf("%f %f\r\n",auto_shoot_data.auto_pit, auto_shoot_data.auto_yaw);
				}
//				else
//				{
//					auto_shoot_data.auto_pit = 0;  
//					auto_shoot_data.auto_yaw = 0; 	 
//				}
				auto_shoot_data.cat_walk = Rx_data[0][6];
				auto_shoot_data.go_point = Rx_data[0][7];
			}
		}
		HAL_UART_Receive_DMA(&huart6, (uint8_t *)Rx_data, pc_count);
		SET_BIT(huart6.Instance->CR1, USART_CR1_IDLEIE);
		DMA1->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;//DMA中断清0
		__HAL_DMA_SET_COUNTER(huart6.hdmarx, pc_count); //重载NDTR位
		__HAL_DMA_ENABLE(huart6.hdmarx);
	} 
}

//返回自瞄控制变量，通过指针传递方式传递信息
const auto_shoot_data_t *get_auto_shoot_point(void)
{
	return &auto_shoot_data;
}
