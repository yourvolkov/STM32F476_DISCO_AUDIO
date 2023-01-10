#include "Interrupt_handlers.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f746xx.h"
#include "main.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
//		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1 ^ HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin));
	}
}
