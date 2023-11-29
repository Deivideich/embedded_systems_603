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
#include "kalman_filter.h"
#include "myprintf.h"
#include "esc.h"
#include "tim.h"
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
struct mpu9250 mpu; //Struct for storing gyro and acc data
struct Stanley stanley; //Struct for Stanley controller
struct KF kf;

osThreadId_t blinkGreenTaskHandle;
const osThreadAttr_t blinkGreenTask_attributes = {
		.name = "blinkGreenTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTaskHandle_attributes = {
		.name = "imuTask",
		.stack_size = 128 * 10,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t escTaskHandle;
const osThreadAttr_t escTaskHandle_attributes = {
		.name = "escTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityAboveNormal,
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
		.stack_size = 128 * 10,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTaskHandle_attributes = {
		.name = "radioTask",
		.stack_size = 128 * 10,
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
void radioTask(void *argument);
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
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  blinkGreenTaskHandle = osThreadNew(blinkGreenTask, NULL, &blinkGreenTask_attributes);
  servoTaskHandle = osThreadNew(servoTask, NULL, &servoTaskHandle_attributes);
  escTaskHandle = osThreadNew(escTask, NULL, &escTaskHandle_attributes);
  radioTaskHandle = osThreadNew(radioTask, NULL, &radioTaskHandle_attributes);
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

void radioTask(void *argument)
{
  float dt = 0.1;
  float real_x = 0.0;
  float real_y = 0.0;
  float init_x = real_x;
  float init_y = real_y;
  float vx = 1.0;
  float vy = 0.0;
  float meas_variance = 0.1 * 0.1;
  float accel_var_x = 0.1;
  float accel_var_y = 0.1;
  initKalman(&kf, init_x, init_y, vx, vy, accel_var_x, accel_var_y);
  int counter = 0;
	for(;;)
	{
    // Modificar tras lectura de radio real!
    vx = 1 * cos(M_PI / 180 * mpu.pose[2]);
    vy = 1 * sin(M_PI / 180 * mpu.pose[2]);
    real_x += dt * vx;
    real_y +=  dt * vy;
    
    osMutexWait(myMutex01Handle, osWaitForever);
    predict(&kf, dt);

    // Reemplazar por if(wifi_received)
    if(counter % 20 == 0)
      update(&kf, real_x, real_y, meas_variance);

    counter++;

    osMutexRelease(myMutex01Handle);
    printf("Real: %3.3f, %3.3f || Estimated: %3.3f, %3.3f \r\n",real_x, real_y, kf.x[0], kf.y[0]);
		osDelay(dt * 1000);
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
  stanleyTaskHandle = osThreadNew(stanleyTask, NULL, &stanleyTaskHandle_attributes);
  float dt = 0.05;
	for(;;)
	{
    osMutexWait(myMutex01Handle, osWaitForever);
		updateData(&mpu, dt, 1); //Printing with func from header file
    osMutexRelease(myMutex01Handle);

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
	  // setPwmS(&escValues);
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
	// setPwmS(&escValues);
	for(;;){
    // printf("ESC Debug \r\n");
    osDelay(1000);
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

	for(;;){
     servoValues.percentage = (int) ( (out[0] + (slope * (stanley.delta - in[0]))));
	//  printf("Y {%u}",servoValues.percentage);
     if(servoValues.percentage != last_steer){
    osMutexWait(myMutex01Handle, osWaitForever);
    // setPwmS(&servoValues);
    osMutexRelease(myMutex01Handle);

	  //  printf("servo {%u},last {%u}",servoValues.percentage, last_steer);
     }
//     osMutexWait(myMutex01Handle, osWaitForever);
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
  
	for(;;){
  vehicle_pos.x = mpu.pose[0];
  vehicle_pos.y = mpu.pose[1];
  psi = mpu.pose[2] * M_PI / 180;
  vel = 0.5;
  osMutexWait(myMutex01Handle, osWaitForever);
  calculateCrosstrackError(&stanley, &vehicle_pos, &p1, &p2);
  setYawAngle(&stanley, psi);
  calculateSteering(&stanley, vel, precision);
  osMutexRelease(myMutex01Handle);
  // printf(" Pose XYZ:");
	for(int i = 0; i<3;i++){
	  //  printf("{%05.1f}",mpu.pose[i]);
	}
	// printf(" Delta: {%05.1f}",stanley.delta * 180 / M_PI);
	// printf("\r\n");
  osDelay(100);
	}
}
/* USER CODE END Application */

