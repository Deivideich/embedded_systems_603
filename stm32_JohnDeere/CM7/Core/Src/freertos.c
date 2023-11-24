/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.h"
#include "myprintf.h"
#include "esc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t blinkGreenTaskHandle;
const osThreadAttr_t blinkGreenTask_attributes = {
		.name = "blinkGreenTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTaskHandle_attributes = {
		.name = "imuTask",
		.stack_size = 128 * 6,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t escTaskHandle;
const osThreadAttr_t escTaskHandle_attributes = {
		.name = "escTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void blinkGreenTask(void *argument);
void imuTask(void *argument);
void escTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  blinkGreenTaskHandle = osThreadNew(blinkGreenTask, NULL, &blinkGreenTask_attributes);
  imuTaskHandle = osThreadNew(imuTask, NULL, &imuTaskHandle_attributes);
  escTaskHandle = osThreadNew(escTask, NULL, &escTaskHandle_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void blinkGreenTask(void *argument)
{
	HAL_GPIO_TogglePin(LD2_yellow_GPIO_Port,LD2_yellow_Pin);
	for(;;)
	{
		HAL_GPIO_TogglePin(LD1_Green_GPIO_Port,LD1_Green_Pin);
		osDelay(500);
	}
}

void imuTask(void *argument)
{
	  char axisLabel[3] = {'X','Y','Z'}; //Var for printing labels
	  struct mpu9250 mpu={AFS_2G,GFS_250DPS}; //Struct for storing gyro and acc data
	  printf("Initiating IMU...\r\n"); //Initiating MPU9250
	  initMPU9250(&mpu, AFS_2G, GFS_250DPS, M_8Hz);

	  printf("Calibrating IMU...\r\n");
	  float accelBias[3], gyroBias[3]; //Calibrating and Printing Biases MPU9250
	  calibrateMPU9250(gyroBias, accelBias);
	  printf("AccBias {");
	  for(int i=0; i<3; i++){
		  printf(" %c %.3f ",axisLabel[i],gyroBias[i]);
	  }
	  printf("} GyroBias{");
	  for(int i=0; i<3; i++){
		  printf(" %c %.3f ",axisLabel[i],accelBias[i]);
	  }
	  printf("}\r\n");

	  printf("Starting IMU...\r\n");
	  float initPose[] = {100,100,90};
	  setPose(&mpu, initPose);
	for(;;)
	{
		updateData(&mpu, 0.1, 1); //Printing with func from header file
		printf("Acc XYZ:");
		for(int i=0;i<3;i++){
		  printf("{%05.3f}",mpu.acc[i]);
		}
		printf(" Gyro XYZ:");
		for(int i = 0; i<3;i++){
		   printf("{%05.1f}",mpu.gyro[i]);
		}
		printf(" Pose XYZ:");
		for(int i = 0; i<3;i++){
		   printf("{%05.1f}",mpu.pose[i]);
		}
		printf("\r\n");
		osDelay(100);
	}
}

void escTask(void *argument){

	for(;;){
	}
}
/* USER CODE END Application */

