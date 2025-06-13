/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h" 
#include "stdlib.h"
#include "stdbool.h"
#include "global.h"
#include "math.h"

#include "front_right.h"
#include "front_left.h"
#include "behind_right.h"
#include "behind_left.h"
//#include "HC_SR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define pi 3.1415
#define p2r pi/2000
//#define Kp 33.54

#define alpha 0.0
//#define Kb 1.49
#define sampletime 0.01

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define UP_LEFT 5
#define UP_RIGHT 6
#define DOWN_LEFT 7
#define DOWN_RIGHT 8
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int16_t  HILIM =100, LOLIM=0;
char Rx_indx, Rx_Buffer[20],Rx_data[2];
int volatile direction=77; 
bool directionset;
Motor motors[4]; 
//uint8_t received_data;
char received_data[25];

uint16_t Distance = 0;
int aviod_sensor[4]; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
int SetVelLow(float CurrentPos, float Pos);
int SetVelMid(float CurrentPos, float Pos, float CurrentVel);
int SetVelHigh(float CurrentPos, float Pos, float CurrentVel);
void calculateWheelSpeeds(double vx, double vy, double wz, double lx, double ly, double r, double* wheelSpeeds);
//uint8_t data [] = "vllll\n";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else 
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif 



void setSpeedAll(float speed ) {
    for (int i = 0; i < 4; i++) {
			if(directionset)
        motors[i].DesiredSpeed = speed;
    }
}

void setSpeed(float speed, int motor1, int motor2) {
        motors[motor1].DesiredSpeed = speed;
        motors[motor2].DesiredSpeed = speed;  
}

	
void forward(void){
	for (int i = 0; i < 4; i++) {
    motors[i].dir = 1;
	}
	directionset = true;
}

void backward(void){
	for (int i = 0; i < 4; i++) {
    motors[i].dir = 0;
	}
	directionset = true;
}

void right(void){
	motors[0].dir = 0;
	motors[1].dir = 1;
	motors[2].dir = 1;
	motors[3].dir = 0;
	directionset = true;

}

void left(void){
	motors[0].dir = 1;
	motors[1].dir = 0;
	motors[2].dir = 0;
	motors[3].dir = 1;
	directionset = true;
}

void up_right(void){
	motors[1].dir = 1;
	motors[2].dir = 1;
	directionset = true;
}

void up_left(void){
	motors[0].dir = 1;
	motors[3].dir = 1;
	directionset = true;
}

void down_right(void){
	motors[1].dir = 0;
	motors[2].dir = 0;
	directionset = true;
}

void down_left(void){
	motors[0].dir = 0;
	motors[3].dir = 0;
	directionset = true;
}

void cw(void){
	motors[0].dir = 0;
	motors[1].dir = 0;
	motors[2].dir = 1;
	motors[3].dir = 1;
	directionset = true;
}
void ccw(void){
	motors[0].dir = 1;
	motors[1].dir = 1;
	motors[2].dir = 0;
	motors[3].dir = 0;
	directionset = true;
}

void config_EXTI(){
	AFIO->EXTICR[0] |= (1<<9)|(1<<13);
	EXTI->IMR |= (1<<2)|(1<<3);
	EXTI->EMR |= (1<<2)|(1<<3);
	EXTI->FTSR |= (1<<2)|(1<<3);
	EXTI->RTSR |= (1<<2)|(1<<3);
	
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
}



	// PI Controller
int PIDVel(int i, float DesiredValue, float CurrentValue) {
    if (motors[i].Ki == 0) motors[i].Kb = 0;
    motors[i].err = DesiredValue - CurrentValue;
    motors[i].up = motors[i].Kp * motors[i].err;

    motors[i].ui = motors[i].ui_p + motors[i].Ki * motors[i].err * sampletime + motors[i].Kb * motors[i].eReset * sampletime;

    motors[i].ui_p = motors[i].ui;

    motors[i].uout = (motors[i].up + motors[i].ui);
    motors[i].uHat = motors[i].uout;

    if (motors[i].uout > HILIM) {
        motors[i].uHat = HILIM;
    } else if (motors[i].uout < LOLIM) {
        motors[i].uHat = LOLIM;
    }

    motors[i].eReset = motors[i].uHat - motors[i].uout;
    return (int)round(motors[i].uHat);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {	// ngat timer 4 tinh van toc
	for (int i = 0; i < 4; i++){ 
	//int i =1;
	if(htim->Instance==TIM4)	// ngat do timer 4	5ms
	{
		motors[i].Cnttmp = motors[i].CntVel;    // so xung trong 10ms  // 1s ngat 200 lan
		motors[i].CntVel = 0;
		motors[i].RealVel = (float)motors[i].Cnttmp*(12000.0/7392.0);			//cnttmp*(60000/N)				//RPM 
		// 200*cnttmp*60 = 6000 ppm
		motors[i].CurVel = motors[i].Cnttmp*25*pi/462;								//rad/s    //(cnttmp*2pi)/7392*0.005
		//motors[i].CurVel = (motors[i].RealVel * 2*pi)/60;
		
		motors[i].pwm = PIDVel (i, fabs(motors[i].DesiredSpeed), motors[i].CurVel);
		//motors[i].pwm = (int)motors[i].DesiredSpeed;
		//if( flag ==1) pwm =0;
		if (motors[i].DesiredSpeed < 0){
			switch (i){
				case 0:
						HAL_GPIO_WritePin(GPIOC,IN1F_Pin, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOC,IN2F_Pin, GPIO_PIN_SET);
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,motors[i].pwm); 	// set pwm
						break;
				case 1:
						HAL_GPIO_WritePin(GPIOC,IN3F_Pin, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOB,IN4F_Pin, GPIO_PIN_SET);
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,motors[i].pwm); 	// set pwm
						break;

				case 2:
						HAL_GPIO_WritePin(GPIOC,IN1B_Pin, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOC,IN2B_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,motors[i].pwm); 	// set pwm
						break;
				case 3:
						HAL_GPIO_WritePin(GPIOB,IN3B_Pin, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOA,IN4B_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,motors[i].pwm); 	// set pwm
						break;					
			}
		}
		else {
			switch (i){
				case 0:
						HAL_GPIO_WritePin(GPIOC,IN1F_Pin, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOC,IN2F_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,motors[i].pwm); 	// set pwm
						break;
				case 1:
						HAL_GPIO_WritePin(GPIOC,IN3F_Pin, GPIO_PIN_SET); 
						HAL_GPIO_WritePin(GPIOB,IN4F_Pin, GPIO_PIN_RESET);
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,motors[i].pwm); 	// set pwm
						break;
	
				case 2:
						HAL_GPIO_WritePin(GPIOC,IN1B_Pin, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOC,IN2B_Pin, GPIO_PIN_SET);
					__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,motors[i].pwm); 	// set pwm
						break;
				case 3:
						HAL_GPIO_WritePin(GPIOB,IN3B_Pin, GPIO_PIN_RESET); 
						HAL_GPIO_WritePin(GPIOA,IN4B_Pin, GPIO_PIN_SET);
					__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,motors[i].pwm); 	// set pwm
						break;					
			}
		}	
		//return;
	}
	if(htim->Instance==TIM5)
	{
			motors[i].tick++;
			if (motors[i].run==0){ 
					motors[i].pwm = 0;
					motors[i].Speedmode =0;
			}
			else if ((motors[i].run==1)&&(motors[i].tick==5)){
				motors[i].tick=0;
			}
	}
}
}


void controlMotors(int direction) {
    switch (direction) {
        case 0:
            forward();
						setSpeedAll(1);
            break;
        case 1:
						
            backward();	
						setSpeedAll(1);			
            break;
        case 2:
						
            right();	
						setSpeedAll(1);
            break;
        case 3:
						
            left();	
						setSpeedAll(1);				
            break;
        case 8:
						
            cw();	
						setSpeedAll(1);
            break;
        case 9:
						
            ccw();
						setSpeedAll(1);
            break;
      
        case 4:
            up_right();
						setSpeed(1, 1, 2); 
            break;
        case 5:
            up_left();
						setSpeed(1, 0, 3);
            break;
        case 6:
            down_right();
						setSpeed(1, 0, 3);
            break;
        case 7:
            down_left();
						setSpeed(1, 1, 2);
            break;
        
        default:
           
            for (int i = 0; i < 4; i++) {
                motors[i].DesiredSpeed = 0;
            }
            directionset = false;
            break;
    }
}


float value[10];
int count = 0;
void process_string(const char *input, float values[], int *count) {
    char buffer[100];
    strcpy(buffer, input);
    char *token;
    char temp[20] = "";
    float value;
    *count = 0;

    token = strtok(buffer, ",");
    while (token != NULL) {
        if (token[0] == '-' || token[0] == '+') {
            if (strlen(temp) > 0) {
                value = atof(temp);
                values[(*count)++] = value;
            }
            strcpy(temp, token);
        } else {
            strcat(temp, token);
        }

        token = strtok(NULL, ",");
    }
    if (strlen(temp) > 0) {
        value = atof(temp);
        values[(*count)++] = value;
    }
}
volatile int uart_tx_flag = 0;
int check=0;
volatile uint8_t uart_rx_flag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
      uart_rx_flag = 1;
			uart_tx_flag = 1;
			check ++;
			HAL_UART_Receive_IT(&huart1, (uint8_t *)received_data, sizeof(received_data));
    }
}
void SendMotorVelocities(void) {
    char Tx_buffer[100]; // Buffer to hold the formatted string
    snprintf(Tx_buffer, sizeof(Tx_buffer), "FR:%.2f,FL:%.2f,BR:%.2f,BL:%.2f\n",
             motors[0].CurVel, motors[1].CurVel, motors[2].CurVel, motors[3].CurVel);

    // Transmit the buffer via UART
    HAL_UART_Transmit(&huart1, (uint8_t*)Tx_buffer, strlen(Tx_buffer), HAL_MAX_DELAY);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */
	RCC->APB2ENR = (1<<3);
	Digital_Input(PB,12);
	Digital_Input(PB,13);
	Digital_Input(PB,14);
	Digital_Input(PB,15);
	
	GPIOB->BSRR |= GPIO_BSRR_BS12_Msk;
	GPIOB->BSRR |= GPIO_BSRR_BS13_Msk;
	GPIOB->BSRR |= GPIO_BSRR_BS14_Msk;
	GPIOB->BSRR |= GPIO_BSRR_BS15_Msk;
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  /* USER CODE BEGIN 2 */
	
	
	config_EXTI();
	//HAL_UART_Receive_IT(&huart1,(uint8_t*)Rx_data,1);
	//HAL_UART_Receive_IT(&huart1, &received_data, 1);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)received_data, sizeof(received_data));
	
	front_right_init();
	front_left_init();
	behind_right_init();
	behind_left_init();
// thong so FR
	  motors[0].Kp = 9.0909;
    motors[0].Ki = 110.4006;
    motors[0].Kb = 12.144;
		// thong so FL
		motors[1].Kp = 10.4545;
    motors[1].Ki = 128.1428;
    motors[1].Kb = 12.2571;
		
				// thong so BR
	  motors[2].Kp = 10.0;
    motors[2].Ki = 112.2448;
    motors[2].Kb = 11.2244;
		// thong so BL
		motors[3].Kp = 10.0;
    motors[3].Ki = 57.4662;
    motors[3].Kb = 5.7466;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//HAL_UART_Transmit(&huart1, data, sizeof(data),10);
	

while (1)
  {
		if (uart_rx_flag) {
			uart_rx_flag = 0;
			
			aviod_sensor[0] = R_GP(PB,12);
			aviod_sensor[1] = R_GP(PB,13);
			aviod_sensor[2] = R_GP(PB,14);
			aviod_sensor[3] = R_GP(PB,15);
			if((aviod_sensor[0] + aviod_sensor[1]) < (aviod_sensor[2] + aviod_sensor[3])){
				motors[0].DesiredSpeed = -3;
				motors[1].DesiredSpeed = 3;
				motors[2].DesiredSpeed = 3;
				motors[3].DesiredSpeed = -3;
				
				HAL_Delay(2000);
			}
			else if((aviod_sensor[0] + aviod_sensor[1]) > (aviod_sensor[2] + aviod_sensor[3])){
				motors[0].DesiredSpeed = 3;
				motors[1].DesiredSpeed = -3;
				motors[2].DesiredSpeed = -3;
				motors[3].DesiredSpeed = 3;
				
				HAL_Delay(2000);
			}
			else if (aviod_sensor[0] == 0 && aviod_sensor[1] ==0 && aviod_sensor[2] == 0 && aviod_sensor[3] == 0){
				motors[0].DesiredSpeed = 0;
				motors[1].DesiredSpeed = 0;
				motors[2].DesiredSpeed = 0;
				motors[3].DesiredSpeed = 0;
			}

			else{
			process_string(received_data,value,&count);
			motors[0].DesiredSpeed = value[1];
			motors[1].DesiredSpeed = value[0];
			motors[2].DesiredSpeed = value[3];
			motors[3].DesiredSpeed = value[2];
			memset(received_data, 0, 25);
			}

			HAL_UART_Receive_IT(&huart1, (uint8_t *)received_data, sizeof(received_data));
			}
			if (uart_tx_flag) {
        uart_tx_flag = 0; 
        SendMotorVelocities();
			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 11;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 23999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1B_Pin|IN2B_Pin|IN3F_Pin|GPIO_PIN_9
                          |IN1F_Pin|IN2F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN4F_Pin|IN3B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN4B_GPIO_Port, IN4B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1B_Pin IN2B_Pin IN3F_Pin IN1F_Pin
                           IN2F_Pin */
  GPIO_InitStruct.Pin = IN1B_Pin|IN2B_Pin|IN3F_Pin|IN1F_Pin
                          |IN2F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4F_Pin IN3B_Pin */
  GPIO_InitStruct.Pin = IN4F_Pin|IN3B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IN4B_Pin */
  GPIO_InitStruct.Pin = IN4B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN4B_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
