#include "front_right.h"



void front_right_init(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	// khoi tao timer 2
	HAL_TIM_Base_Start_IT(&htim4);             // Kh?i t?o timer 4 cho ng?t
  HAL_TIM_Base_Start_IT(&htim5);             // Kh?i t?o timer 3 cho ng?t
}




/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI4_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  handler_encoder_EXTI(0,4,6);
	EXTI->PR |= EXTI_PR_PR4;
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}
