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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h"
#include "math.h"
#include "stdbool.h"

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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
void mpu9250_write_reg(uint8_t reg, uint8_t data);
void calibrate_mag();
void init_mag();
bool read_mag(int16_t*);

float mag_bias[3] = {0., 0., 0.};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
float mag_scale[3] = {1., 1., 1.};
float mag_bias_factory[3] = {0.0, 0.0, 0.0};
float magnetic_declination = -7.51;  // Japan, 24th June
float mag_resolution = 10. * 4912. / 32760.0;   // scale resolutions per LSB for the sensors

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  int16_t acc_x, acc_y, acc_z;
  int16_t gyro_x, gyro_y, gyro_z;
  //int16_t mag_x, mag_y, mag_z;
  uint8_t imu_data[14];
  int16_t mag_count[3] = {0, 0, 0};  // Stores the 16-bit signed magnetometer sensor output
  mpu9250_write_reg(27, 0x00);
  HAL_Delay(100);
  mpu9250_write_reg(28, 0x08);
  HAL_Delay(100);
  float a_conv = 4.0 / 32768;
  float g_conv = 250.0 / 32768.0;
  float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z;
  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Starting !\r\n");
  calibrate_mag();
  while (1)
  {
	  HAL_Delay(5);
	  mpu9250_read_reg(59, imu_data, sizeof(imu_data));
	  acc_x = ((int16_t)imu_data[0]<<8) + imu_data[1];
	  acc_y = ((int16_t)imu_data[2]<<8) + imu_data[3];
	  acc_z = ((int16_t)imu_data[4]<<8) + imu_data[5];

	  mpu9250_read_reg(67, imu_data, sizeof(imu_data));
	  gyro_x = ((int16_t)imu_data[0]<<8) + imu_data[1];
	  gyro_y = ((int16_t)imu_data[2]<<8) + imu_data[3];
	  gyro_z = ((int16_t)imu_data[4]<<8) + imu_data[5];

	  if(read_mag(mag_count)){
		  m_x=(float)(mag_count[0]*mag_resolution*mag_bias_factory[0]-mag_bias[0])*mag_scale[0];  // get actual magnetometer value, this depends on scale being set
		  m_y=(float)(mag_count[1]*mag_resolution*mag_bias_factory[1]-mag_bias[1])*mag_scale[1];
		  m_z=(float)(mag_count[2]*mag_resolution*mag_bias_factory[2]-mag_bias[2])*mag_scale[2];
	  } else {
		  printf("huh\r\n");
	  }

	  a_x = (float)(acc_x *  a_conv - 1);
	  a_y = (float)(acc_y *  a_conv + 0.27);
	  a_z = (float)(acc_z *  a_conv);
	  g_x = (float)(gyro_x *  g_conv);
	  g_y = (float)(gyro_y *  g_conv);
	  g_z = (float)(gyro_z *  g_conv);

//	  mpu9250_read_reg(67, imu_data, sizeof(imu_data));
//	  mag_x = ((int16_t)imu_data[0]<<8) + imu_data[1];
//	  mag_y = ((int16_t)imu_data[2]<<8) + imu_data[3];
//	  mag_z = ((int16_t)imu_data[4]<<8) + imu_data[5];
//	  m_x = (float)(mag_x *  m_conv);
//	  m_y = (float)(mag_y *  m_conv);
//	  m_z = (float)(mag_z *  m_conv);

//  m[0] = (float)(mag_count[0] * mag_resolution * mag_bias_factory[0] - mag_bias[0] * bias_to_current_bits) * mag_scale[0];  // get actual magnetometer value, this depends on scale being set

//	  printf("Acc{x: %.3f, y: %.3f, z:%.3f}, Gyro{x: %.3f, y: %.3f, z:%.3f}\r\n",a_x, a_y, a_z,g_x, g_y, g_z);
	  printf("Mag{x: %.3f, y: %.3f, z:%.3f}\r\n",m_x, m_y, m_z);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_07DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void mpu9250_write_reg(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t temp_data = 0x80|reg;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &temp_data , 1, 100);
	HAL_SPI_Receive(&hspi1, data, len, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}

bool read_mag(int16_t* destination) {
	const uint8_t st1;
	mpu9250_read_reg(0x02, st1, sizeof(st1));
	if (st1 & 0x01) {                                                    // wait for magnetometer data ready bit to be set
		uint8_t raw_data[7];
		// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		mpu9250_read_reg(0x03, &raw_data[0], 7);
		if ((st1 & 0x02) != 0){                                      // check if data is not skipped
			return false;                                            // this should be after data reading to clear DRDY register
		} else {
			printf("!st1.2\r\n");
		}
		uint16_t c = raw_data[6];                                         // End data read by reading ST2 register
		if (!(c & 0x08)) {                                               // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)raw_data[1] << 8) | raw_data[0];  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)raw_data[3] << 8) | raw_data[2];  // Data stored as little Endian
			destination[2] = ((int16_t)raw_data[5] << 8) | raw_data[4];
			return true;
		} else {
			printf("!c\r\n");
		}
	} else {
		printf("!st1.1\r\n");
	}
	return false;
}

void init_mag(){
	uint8_t raw_data[3];                            // x/y/z gyro calibration data stored here
	mpu9250_write_reg(11, 0x00);
	HAL_Delay(10);
	mpu9250_write_reg(11, 0x0F);
	HAL_Delay(10);
	mpu9250_read_reg(16, raw_data[0], sizeof(raw_data));
	mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.;
	mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
	mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
	mpu9250_write_reg(11, 0x00);
	HAL_Delay(10);
	mpu9250_write_reg(11,(uint8_t)0x06);
	HAL_Delay(10);
}

void calibrate_mag() {
    printf("Magnetometer calibration started...\r\n\n");
    init_mag();
    HAL_Delay(4000);
	// shoot for ~fifteen seconds of mag data
	uint16_t sample_count = 0;
	sample_count = 1500;    // at 100 Hz ODR, new mag data is available every 10 ms
	int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767};
	int16_t mag_min[3] = {32767, 32767, 32767};
	int16_t mag_temp[3] = {0, 0, 0};
	for (uint16_t ii = 0; ii < sample_count; ii++) {
		read_mag(mag_temp);  // Read the mag data
		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		HAL_Delay(12);   // at 100 Hz ODR, new mag data is available every 10 ms
	}

	// Get hard iron correction
	bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	float bias_resolution = 10. * 4912. / 32760.0;
	mag_bias[0] = (float)bias[0] * bias_resolution * mag_bias_factory[0];  // save mag biases in G for main program
	mag_bias[1] = (float)bias[1] * bias_resolution * mag_bias_factory[1];
	mag_bias[2] = (float)bias[2] * bias_resolution * mag_bias_factory[2];

	// Get soft iron correction estimate
	//*** multiplication by mag_bias_factory added in accordance with the following comment
	//*** https://github.com/kriswiner/MPU9250/issues/456#issue-836657973
	scale[0] = (float)(mag_max[0] - mag_min[0]) * mag_bias_factory[0] / 2;  // get average x axis max chord length in counts
	scale[1] = (float)(mag_max[1] - mag_min[1]) * mag_bias_factory[1] / 2;  // get average y axis max chord length in counts
	scale[2] = (float)(mag_max[2] - mag_min[2]) * mag_bias_factory[2] / 2;  // get average z axis max chord length in counts

	float avg_rad = scale[0] + scale[1] + scale[2];
	avg_rad /= 3.0;

	mag_scale[0] = avg_rad / ((float)scale[0]);
	mag_scale[1] = avg_rad / ((float)scale[1]);
	mag_scale[2] = avg_rad / ((float)scale[2]);
    init_mag();
    printf("Magnetometer calibrated!\r\n");
}
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
