/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h"
#include "esc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Starting Timer 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Staring Timer 3
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //@notes: Variables for Pulse Width Modulatio for ESC (motor) and servo.
  //Esc variables are set for backwards motion (depending on polarity).
  int minPulseWidthESC = 1000; //Range for variables for PWM (ESC - backwards)
  int maxPulseWidthESC = 1500;
  int minPulseWidthServo = 1000; //Range for right and left Servo
  int maxPulseWidthServo = 2000;
  unsigned int pwmPeriod = 20000;
  int resolution = 100;



  struct escValues escValues = {htim2, minPulseWidthESC, //Struct Containing all
		  maxPulseWidthESC, pwmPeriod, resolution};	  		//PWM Variables for ESC

  struct escValues servoValues = {htim3, minPulseWidthServo, //Struct Containing all
		  maxPulseWidthServo, pwmPeriod, resolution};	  	 //PWM Variables for Servo

  //@notes: ESC needs to start gradually
  //after starting gradually, it will respond to pwm. If given a pwm
  //without this setup, it doesn't respond.
  printf("Starting Setup...\r\n"); // Setup
  int i = 100;
  int dt = 1;
  do{
	  //setPwm(htim2, minPulseWidth, maxPulseWidth, pwmPeriod, resolution, i);
	  setPwmS(&escValues);
	  escValues.percentage=(unsigned int)i;
	  HAL_Delay(10);
	  i=i-dt;

  }while(i > 50);
  printf("End Setup\r\n");

  uint8_t inputESC[3], inputServo[3]; //Variables for Receiving UART
  int inputIntESC, inputIntServo;

  while (1)
  {

	  //@note: Test for checking motor with ESC. Receiving from UART
	  //values from 000 to 100. Change minPulseWidth, maxPulseWidth
	  //pwmPeriod, resoution and escValues.percentage accordingly to
	  //your needs.
	  printf("PWM: ");
	  while(HAL_UART_Receive(&huart3, inputESC, 3, HAL_MAX_DELAY)); //Receiving from UART ESC
	  printf("Received PWM: {%c%c%c} \r\n",inputESC[0],inputESC[1],inputESC[2]);
	  printf("\nServo: ");
	  while(HAL_UART_Receive(&huart3, inputServo, 3, HAL_MAX_DELAY)); //Receiving from UART Servo
	  printf("Received PWM: {%c%c%c} \r\n",inputServo[0],inputServo[1],inputServo[2]);

	  for(int i = 0 ; i < 4 ; i++){//Deleting 0 to convert from ASCII to DEC
		  inputESC[i]-='0';
		  inputServo[i]-='0';
	  }

	  inputIntESC = (inputESC[0]*100 + inputESC[1]*10 + inputESC[2]);//Converting to INT ESC
	  inputIntServo = (inputServo[0]*100 + inputServo[1]*10 + inputServo[2]);//Converting to INT Servo

	  if(inputIntESC < 101 && inputIntESC >= 0){//Limiting input ESC
		  escValues.percentage = inputIntESC;
	  }
	  if(inputIntServo < 101 && inputIntServo >= 0){//Limiting input Servo
		  servoValues.percentage = inputIntServo;
	  }

	  //Setting PWM value
	  //setPwm(htim2, minPulseWidth, maxPulseWidth, pwmPeriod, resolution, percentage);
	  setPwmS(&escValues);
	  //HAL_Delay(100);
	  setPwmS(&servoValues);

	  //@note: Printing percentage, pulse width, set min and set max
	  printf("ESC   - P: %3d PW: %.5f (min: %.5f max: %.5f)\r\n",escValues.percentage,escValues.pulseWidth,escValues.minPulseWidth/1e6,escValues.maxPulseWidth/1e6);
	  printf("Servo - P: %3d PW: %.5f (min: %.5f max: %.5f)\r\n",servoValues.percentage,servoValues.pulseWidth,servoValues.minPulseWidth/1e6,servoValues.maxPulseWidth/1e6);


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
