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
#include "stanley_controller.h"
#include "myprintf.h"
#include "esc.h"
#include "tim.h"
#include "MY_NRF24.h"
#include "spi.h"
#include "usart.h"
#include "fdcan.h"
#include "myCAN.h"
//#include
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
struct mpu9250 mpu={AFS_2G,GFS_250DPS}; //Struct for storing gyro and acc data
struct Stanley stanley; //Struct for Stanley controller
uint8_t myRxData[32]; //Var for storing data from radio(camera)
struct dataCam dataCam = {0,0,0};
float speed; //Var for speed(CAN)

osThreadId_t blinkGreenTaskHandle;
const osThreadAttr_t blinkGreenTask_attributes = {
		.name = "blinkGreenTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTaskHandle_attributes = {
		.name = "imuTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t escTaskHandle;
const osThreadAttr_t escTaskHandle_attributes = {
		.name = "escTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTaskHandle_attributes = {
		.name = "servoTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t stanleyTaskHandle;
const osThreadAttr_t stanleyTaskHandle_attributes = {
		.name = "stanleyTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t wirelessTaskHandle;
const osThreadAttr_t wirelessTaskHandle_attributes = {
		.name = "wirelessTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t canTaskHandle;
const osThreadAttr_t canTaskHandle_attributes = {
		.name = "canTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void blinkGreenTask(void *argument);
void imuTask(void *argument);
void escTask(void *argument);
void servoTask(void *argument);
void stanleyTask(void *argument);
void wirelessTask(void *argument);
void canTask(void *argument);
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
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // blinkGreenTaskHandle = osThreadNew(blinkGreenTask, NULL, &blinkGreenTask_attributes);
  escTaskHandle = osThreadNew(escTask, NULL, &escTaskHandle_attributes); //First
  // servoTaskHandle = osThreadNew(servoTask, NULL, &servoTaskHandle_attributes);
  // wirelessTaskHandle = osThreadNew(wirelessTask, NULL, &wirelessTaskHandle_attributes);
   // canTaskHandle = osThreadNew(canTask, NULL, &canTaskHandle_attributes);
  //
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
    osDelay(10000);
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
	  float initPose[] = {0,0,0};
	  setPose(&mpu, initPose);

	  wirelessTaskHandle = osThreadNew(wirelessTask, NULL, &wirelessTaskHandle_attributes);

    float dt = 0.05;
	for(;;)
	{
		updateData(&mpu, 0.1, dt); //Printing with func from header file
		// printf("Acc XYZ:");
		// for(int i=0;i<3;i++){
		//   printf("{%05.3f}",mpu.acc[i]);
		// }
		// printf(" Gyro XYZ:");
		// for(int i = 0; i<3;i++){
		//    printf("{%05.1f}",mpu.gyro[i]);
		// }
    // osMutexWait(myMutex01Handle, osWaitForever);
		// printf(" Pose XYZ:");
		// for(int i = 0; i<3;i++){
		//    printf("{%05.1f}",mpu.pose[i]);
		// }
		// printf("\r\n");
    // osMutexRelease(myMutex01Handle);
		osDelay(dt*1000);
	}
}

void escTask(void *argument){
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Staring Timer 3
	int minPulseWidthEsc = 1000; //Range for right and left Esc
	int maxPulseWidthEsc = 1500;
	unsigned int pwmPeriod = 20000;
	int resolution = 100;
	struct escValues escValues = {htim2, minPulseWidthEsc, //Struct Containing all
	maxPulseWidthEsc, pwmPeriod, resolution};	  	 //PWM Variables for Esc

  // Calibration
  int i = 100;
  int dt = 1;
  do{
	  setPwmS(&escValues);
	  escValues.percentage=(unsigned int)i;
	  HAL_Delay(10);
	  i=i-dt;

  }while(i > 50);

  minPulseWidthEsc = 1500;
  maxPulseWidthEsc = 2000;
  escValues.maxPulseWidth = maxPulseWidthEsc;
  escValues.minPulseWidth = minPulseWidthEsc;

  imuTaskHandle = osThreadNew(imuTask, NULL, &imuTaskHandle_attributes);

	escValues.percentage = 30;
	setPwmS(&escValues);
	for(;;){
    osDelay(100);
	}
}

void servoTask(void *argument){
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Staring Timer 3
  int minPulseWidthServo = 1000; //Range for right and left Servo
  int maxPulseWidthServo = 2000;
  unsigned int pwmPeriod = 20000;
  int resolution = 100;
  struct escValues servoValues = {htim3, minPulseWidthServo, //Struct Containing all
  maxPulseWidthServo, pwmPeriod, resolution};	  	 //PWM Variables for Servo

  float in[2] = {stanley.sat[1], stanley.sat[0]}; // min, max delta values
  float out[2] = {90, 0}; // min, max percentage values
  float slope = (float)(out[1] - out[0]) / (in[1] - in[0]);

  uint8_t last_steer = 0;

  servoValues.percentage = 50;
  setPwmS(&servoValues);

  canTaskHandle = osThreadNew(canTask, NULL, &canTaskHandle_attributes);

	for(;;){
     servoValues.percentage = (int) ( (out[0] + (slope * (stanley.delta - in[0]))));
//     osMutexWait(myMutex01Handle, osWaitForever);
//	 printf("Y {%u}",servoValues.percentage);
     if(servoValues.percentage != last_steer){
       setPwmS(&servoValues);
//	   printf("servo {%u},last {%u}",servoValues.percentage, last_steer);
     }
//     osMutexRelease(myMutex01Handle);
     last_steer = servoValues.percentage;
    osDelay(100);
	}
}

void stanleyTask(void *argument){
  float st_saturation_limits[] = {21.4 * M_PI / 180, -21.4 * M_PI / 180}; // Saturation array
  float st_k = 5; // Gain
  float st_k_soft = 0.01; // Soft gain
  uint8_t precision = 10; // Result's float resolution

  // Control signals
  float vel = 0;

  // Vehicle pose
  struct Point vehicle_pos = {0, 0};
  float psi = 0;

  // Path
  struct Point p1;
  struct Point p2;

  p1.x = 0;
  p1.y = 0;
  p2.x = 10;
  p2.y = 0;

  initStanley(&stanley,st_saturation_limits, st_k, st_k_soft);
  servoTaskHandle = osThreadNew(servoTask, NULL, &servoTaskHandle_attributes);
  
	for(;;){
    vehicle_pos.x = mpu.pose[0];
    vehicle_pos.y = mpu.pose[1];
    psi = mpu.pose[2] * M_PI / 180;
    vel = 0.5;
    calculateCrosstrackError(&stanley, &vehicle_pos, &p1, &p2);
    setYawAngle(&stanley, psi);
    calculateSteering(&stanley, vel, precision);
//    osMutexWait(myMutex01Handle, osWaitForever);
//    printf(" Pose XYZ:");
//	for(int i = 0; i<3;i++){
//	   printf("{%05.1f}",mpu.pose[i]);
//	}
//	printf(" Delta: {%05.1f}",stanley.delta * 180 / M_PI);
//	printf("\r\n");
//    osMutexRelease(myMutex01Handle);
    osDelay(50);
	}
}

void wirelessTask(void *argument){
	uint64_t RxpipeAddrs = 0x11223344AA; //Address of sender

	osMutexWait(myMutex01Handle, osWaitForever); //Setting up Radio
	mySetupNRF24(nrf_CSN_PORT, nrf_CSN_PIN, nrf_CE_PIN,
			  hspi2,huart3,52, RxpipeAddrs, 1);
	osMutexRelease(myMutex01Handle);

	int maxX = 100; //Max and min values
	int minX = 0;
	int maxY = 100;
	int minY = 0;
	int maxA = 360;
	int minA = 360;

	stanleyTaskHandle = osThreadNew(stanleyTask, NULL, &stanleyTaskHandle_attributes);

	for(;;){
		myReadData(myRxData);
		dataCam.x = (uint16_t)myRxData[0] << 8 | myRxData[1];
		dataCam.y = (uint16_t)myRxData[2] << 8 | myRxData[3];
		dataCam.theta = (uint16_t)myRxData[4] << 8 | myRxData[5];

//		dataCam.x = dataCam.x < minX ? minX : dataCam.x;
//		dataCam.x = dataCam.x > maxX ? maxX : dataCam.x;
//		dataCam.y = dataCam.y < minY ? minY : dataCam.y;
//		dataCam.y = dataCam.y > maxY ? maxY : dataCam.y;
//		dataCam.theta = dataCam.theta < minA ? minA : dataCam.theta;
//		dataCam.theta = dataCam.theta > maxA ? maxA : dataCam.theta;
//		for(int i=0;i<6;i++){
//			printf("%x ",myRxData[i]);
//		}
		printf("%x%x %x%x %x%x",myRxData[0],myRxData[1],myRxData[2],myRxData[3],myRxData[4],myRxData[5]);
		printf("\r\n");
		printf("X:%d Y:%d Angle:%d\r\n",dataCam.x,dataCam.y,dataCam.theta);
		printf("\r\n");
		osDelay(1000);
	}
}

void canTask(void *argument){
	FDCAN_RxHeaderTypeDef RxHeader;
	union BytesFloat bf;
	uint8_t RxData[8];
//	float speed=1.3;
	uint8_t m;

	printf("Starting CAN...\r\n");
	for(;;){
		m = readSpeed(&hfdcan1, &RxHeader, bf, RxData, &speed);
//		printf("S:%d {",m);
//		for(int i=0;i<8;i++){
//			printf("%x",RxData[i]);
//		}
//		printf("} %f \r\n",speed);
		osDelay(100);

	}
}
/* USER CODE END Application */

