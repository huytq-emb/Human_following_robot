#ifndef GLOBAL_H
#define GLOBAL_H
#include "stm32f1xx_hal.h"
#include "math.h"
#include <stdbool.h>

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define pi 3.1415
#define p2r pi/2000
//#define Kp 33.54

#define alpha 0.0
//#define Kb 1.49
#define sampletime 0.01
extern int16_t  HILIM , LOLIM;
typedef struct {
    int32_t PosCnt, Cnttmp, speed; 
    int16_t CountValue; 
    uint16_t AngPos1, AngPos0, CntVel; 
    uint8_t PreviousState, Speedmode, tick, flag; 
    bool run, dir;
    float CurPos, DesiredPos, CurVel, RealVel; 
    char Rx_indx, Rx_Buffer[20], Rx_data[2]; 
    volatile float Kp, Ki, Kd, DesiredSpeed, Kb; 
    int pwm; 
    volatile int dem; 
    float ui_p, err, up, ud, ui, uHat, eReset, uout;
    //int uout; 
} Motor; 

// Khai báo m?ng d?ng co
extern Motor motors[4];  // M?ng ch?a 4 d?ng co

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5; 


void handler_encoder_EXTI(unsigned int i, unsigned int pin1, unsigned int pin2);
//int PIDVel(int i, float DesiredValue, float CurrentValue);
// Khai báo các bi?n toàn c?c
//extern float sampletime; // Th?i gian m?u
//extern int16_t HILIM;        // Gi?i h?n cao
//extern int16_t LOLIM;       // Gi?i h?n th?p

#endif  // GLOBAL_H
