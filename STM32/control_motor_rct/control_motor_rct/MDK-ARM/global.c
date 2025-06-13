#include "global.h"


void EXTI9_5_IRQHandler(void)	// doc encoder	
{ 
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	if(EXTI->PR & EXTI_PR_PR6){
  handler_encoder_EXTI(0,4,6);
	EXTI->PR |= EXTI_PR_PR6;
	}
	if(EXTI->PR & EXTI_PR_PR5){
  handler_encoder_EXTI(3,5,7);
	EXTI->PR |= EXTI_PR_PR5;
	}
	if(EXTI->PR & EXTI_PR_PR7){
  handler_encoder_EXTI(3,5,7);
	EXTI->PR |= EXTI_PR_PR7;
	}
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}

void handler_encoder_EXTI(unsigned int i, unsigned int pin1, unsigned int pin2){
	unsigned char State0;
	State0 = (State0<<1) | ((GPIOB->IDR >> pin1) & 0x01);
	State0 = (State0<<1) | ((GPIOB->IDR >> pin2) & 0x01);
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

  /* USER CODE BEGIN EXTI4_IRQn 1 */
}

