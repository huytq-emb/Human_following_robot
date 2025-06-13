#include "behind_right.h"



void behind_right_init(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	// khoi tao timer 2
	HAL_TIM_Base_Start_IT(&htim4);             // Kh?i t?o timer 4 cho ng?t
  HAL_TIM_Base_Start_IT(&htim5);             // Kh?i t?o timer 3 cho ng?t
}


void EXTI2_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
int i =2;
unsigned char State0;
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
	State0 = State0&0x03;
	switch (State0) {
		case 0:
			if(motors[i].PreviousState==1) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		case 1:
			if(motors[i].PreviousState==3) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		case 2:
			if(motors[i].PreviousState==0) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		case 3:
			if(motors[i].PreviousState==2) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		}
	motors[i].PreviousState = State0;
	motors[i].CntVel++;
	if (motors[i].CountValue>=7392) {
		motors[i].flag = 1;
		motors[i].CountValue = 0;
		//PosCnt++;
	}
	else if	(motors[i].CountValue<=-7392) {
		motors[i].flag = 1;
		motors[i].CountValue = 0;
		//PosCnt--;
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
int i =2;
unsigned char State0;
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
	State0 = State0&0x03;
	switch (State0) {
		case 0:
			if(motors[i].PreviousState==1) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		case 1:
			if(motors[i].PreviousState==3) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		case 2:
			if(motors[i].PreviousState==0) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		case 3:
			if(motors[i].PreviousState==2) motors[i].CountValue++;
			else motors[i].CountValue--;
		break;
		}
	motors[i].PreviousState = State0;
	motors[i].CntVel++;
	if (motors[i].CountValue>=7392) {
		motors[i].flag = 1;
		motors[i].CountValue = 0;
		//PosCnt++;
	}
	else if	(motors[i].CountValue<=-7392) {
		motors[i].flag = 1;
		motors[i].CountValue = 0;
		//PosCnt--;
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}
