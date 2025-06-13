#include "behind_left.h"



void behind_left_init(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// khoi tao timer 2
	HAL_TIM_Base_Start_IT(&htim4);             // Kh?i t?o timer 4 cho ng?t
  HAL_TIM_Base_Start_IT(&htim5);             // Kh?i t?o timer 3 cho ng?t
}




/**
* @brief This function handles EXTI line3 interrupt.
*/

