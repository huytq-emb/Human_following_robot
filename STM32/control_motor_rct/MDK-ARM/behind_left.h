#ifndef BEHIND_LEFT_H
#define BEHIND_LEFT_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "global.h"
#define pi 3.1415
#define p2r pi/2000
//#define Kp 33.54

#define alpha 0.0
//#define Kb 1.49
#define sampletime 0.01

extern int16_t  HILIM , LOLIM;
////int32_t PosCnt, Cnttmp, speed;
//extern int16_t  HILIM, LOLIM;
////extern uint16_t AngPos1, AngPos0, CntVel;
////extern uint8_t PreviousState, Speedmode, tick, flag;
//extern volatile bool run;
//extern int16_t CountValue;
//extern float CurPos, DesiredPos, CurVel, RealVel;
////extern volatile float Kp, Ki, Kd, DesiredSpeed, Kb;
//extern volatile float DesiredSpeed;
////extern int pwm;
 

extern Motor motors[4];


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

void behind_left_init(void);
void EXTI9_5_IRQHandler(void);
//int PIDVel(float DesiredValue, float CurrentValue);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* BEHIND_LEFT_H */
