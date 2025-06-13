#include "HC_SR04.h"

extern TIM_HandleTypeDef htim1;

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
extern uint16_t Distance; // 

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOC

#define MAX_DISTANCE 150  // Gi?i h?n kho?ng cách t?i da (150 cm)

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // N?u ng?t t? channel 1
    {
        if (Is_First_Captured == 0)  // L?n b?t d?u
        {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Ð?c giá tr? l?n 1
            Is_First_Captured = 1;  // Ðánh d?u l?n b?t d?u
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);  // Ð?i d?u hi?u b?t
        }
        else if (Is_First_Captured == 1)  // L?n b?t th? 2
        {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Ð?c giá tr? l?n 2
            __HAL_TIM_SET_COUNTER(htim, 0);  // Reset b? d?m

            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;
            } else {
                Difference = (0xffff - IC_Val1) + IC_Val2;
            }

            // Ki?m tra n?u kho?ng cách vu?t quá ngu?ng t?i da (150cm)
            Distance = (Difference * 0.034 / 2);  // Tính kho?ng cách (cm)

            if (Distance > MAX_DISTANCE) {
                Distance = 0;  // N?u quá 150cm thì không có v?t c?n
            }

            Is_First_Captured = 0;  // Ð?t l?i tr?ng thái
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);  // Ð?i l?i d?u hi?u b?t
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);  // T?t ng?t
        }
    }
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

#define SAMPLE_COUNT 5  // So lan do de tinh gia tri trung binh
#define NO_OBJECT_THRESHOLD 3  // Nguong so lan do de xac dinh khong co vat the

uint16_t Distance_Avg = 0;  // Bien luu tru khoang cach trung binh
uint8_t No_Object_Counter = 0;  // Dem so lan khong co vat the

void Calculate_Average_Distance() {
    uint32_t sum = 0;
    uint8_t valid_measurements = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        HCSR04_Read();  // Doc gia tri khoang cach
        HAL_Delay(200);  // Doi 200 ms de xu ly tin hieu Echo

        // Kiem tra neu khoang cach hop le (khong bang 0 hoac qua lon)
        if (Distance > 0 && Distance <= MAX_DISTANCE) {
            sum += Distance;  // Cong don ket qua
            valid_measurements++;  // Dem so phep do hop le
        } else {
            // Neu khong co vat the hoac khoang cach khong hop le, tang bo dem
            No_Object_Counter++;
        }
    }

    // Neu co it nhat 1 phep do hop le, tinh gia tri trung binh
    if (valid_measurements > 0) {
        Distance_Avg = sum / valid_measurements;
        No_Object_Counter = 0;  // Reset bo dem khi co phep do hop le
    } else {
        // Neu tat ca cac phep do deu khong hop le, kiem tra bo dem
        if (No_Object_Counter >= NO_OBJECT_THRESHOLD) {
            Distance_Avg = 0;  // Neu khong co vat the trong 3 phep do lien tiep, dat ve 0
        }
    }
}



