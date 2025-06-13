#include "front_left.h"




void front_left_init(){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// khoi tao timer 2
	HAL_TIM_Base_Start_IT(&htim4);             // Kh?i t?o timer 4 cho ng?t
  HAL_TIM_Base_Start_IT(&htim5);        // Kh?i t?o timer 3 cho ng?t
}

void EXTI1_IRQHandler(void)	// doc encoder	
{
  handler_encoder_EXTI(1,0,1);
	EXTI->PR |= EXTI_PR_PR1;
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI0_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  handler_encoder_EXTI(1,0,1);
	EXTI->PR |= EXTI_PR_PR0;
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}


