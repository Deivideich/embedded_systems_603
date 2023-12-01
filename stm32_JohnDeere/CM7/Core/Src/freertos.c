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
#include "math.h"
#include "myCAN.h"
#include "waypoints.h"
#include "kalman_filter.h"
#include "pi.h"
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
float speed = 0; //Var for speed(CAN)
struct KF kf;
struct PI pi;
struct waypoint_buffer wp_buf;
float initPose[] = {0,0,0};
float st_saturation_limits[] = {21.4 * M_PI / 180, -21.4 * M_PI / 180}; // Saturation array
bool reached = false;
int imuReady = 0;

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
		.stack_size = 128 * 16,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t radioTaskHandle;
const osThreadAttr_t radioTaskHandle_attributes = {
		.name = "radioTask",
		.stack_size = 128 * 10,
		.priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t wpTaskHandle;
const osThreadAttr_t wpTaskHandle_attributes = {
		.name = "wpTask",
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
// goto
void blinkGreenTask(void *argument);
void canTask(void *argument);
void escTask(void *argument);
void imuTask(void *argument);
void radioTask(void *argument);
void servoTask(void *argument);
void stanleyTask(void *argument);
void wirelessTask(void *argument);
void wpTask(void *argument);
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
//  blinkGreenTaskHandle = osThreadNew(blinkGreenTask, NULL, &blinkGreenTask_attributes);

  servoTaskHandle = osThreadNew(servoTask, NULL, &servoTaskHandle_attributes);
   escTaskHandle = osThreadNew(escTask, NULL, &escTaskHandle_attributes);
  radioTaskHandle = osThreadNew(radioTask, NULL, &radioTaskHandle_attributes);
  wpTaskHandle = osThreadNew(wpTask, NULL, &wpTaskHandle_attributes);

  imuTaskHandle = osThreadNew(imuTask, NULL, &imuTaskHandle_attributes);
  // stanleyTaskHandle = osThreadNew(stanleyTask, NULL, &stanleyTaskHandle_attributes);
  // wirelessTaskHandle = osThreadNew(wirelessTask, NULL, &wirelessTaskHandle_attributes);
   // canTaskHandle = osThreadNew(canTask, NULL, &canTaskHandle_attributes);

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

void canTask(void *argument){
	FDCAN_RxHeaderTypeDef RxHeader;
	union BytesFloat bf;
	uint8_t RxData[8];
//	float speed=1.3;
	uint8_t m;
	float loc_speed;


	printf("Starting CAN...\r\n");
	for(;;){
		// osMutexWait(myMutex01Handle, osWaitForever);
		readSpeed(&hfdcan1, &RxHeader, bf, RxData, &m, &loc_speed);
    // osMutexWait(myMutex01Handle, osWaitForever);
		speed = loc_speed / 10 * imuReady;
		if(speed < 0)
			speed*=-1;
		// osMutexRelease(myMutex01Handle);
		// osMutexRelease(myMutex01Handle);
//		printf("S:%d {",m);
//		for(int i=0;i<8;i++){
//			printf("%x",RxData[i]);
//		}
////		speed = (float)loc_sp;
//		printf("} %3.3f, CAN: %3.3f \r\n",speed, speed);

		osDelay(100);
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

  float dtTask = 0.1;

  initPI(&pi, dtTask, 190.0, 80.0, 10.0, 100, 10);

  minPulseWidthEsc = 1500;
  maxPulseWidthEsc = 2000;
  escValues.maxPulseWidth = maxPulseWidthEsc;
  escValues.minPulseWidth = minPulseWidthEsc;

  // imuTaskHandle = osThreadNew(imuTask, NULL, &imuTaskHandle_attributes);

//  escValues.percentage = 0;
//  setPwmS(&escValues);

  updateReferences(&pi, 0.5);
  saturateManipulation(&pi, speed);
//  escValues.percentage = (unsigned int)pi.u;
//  escValues.percentage = 0;
//  setPwmS(&escValues);

	for(;;){

    if(reached){
      escValues.percentage = 0;
      setPwmS(&escValues);
    } else {
      saturateManipulation(&pi, speed);
      //      escValues.percentage = (unsigned int)pi.u;
      escValues.percentage = (unsigned int)50;
      setPwmS(&escValues);
    }
    osDelay(1000 * dtTask);
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
	  float pose[] = {0,0,0};
	  setPose(&mpu, initPose);
	  int counter = 1;

		// wirelessTaskHandle = osThreadNew(wirelessTask, NULL, &wirelessTaskHandle_attributes);
	  stanleyTaskHandle = osThreadNew(stanleyTask, NULL, &stanleyTaskHandle_attributes);
    float dt = 0.1;
    imuReady = 1;
	for(;;)
	{
    // osMutexWait(myMutex01Handle, osWaitForever);
		updateData(&mpu, dt, speed); //Printing with func from header file
		// osMutexRelease(myMutex01Handle);
		if(counter % 10 == 0){
			// pose[2] = dataCam.psi;
			// setPose(&mpu, pose);
		}
		counter++;

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
		osDelay(dt * 1000);
	}
}

void radioTask(void *argument) {
  float dt = 0.1;
  float real_x = 0.0;
  float real_y = 0.0;
  float init_x = real_x;
  float init_y = real_y;
  float vx = 0.1;
  float vy = 0.1;
  float meas_variance = 0.1 * 0.1;
  float accel_var_x = 0.1;
  float accel_var_y = 0.1;
  initKalman(&kf, init_x, init_y, vx, vy, accel_var_x, accel_var_y);
  int counter = 0;
	for(;;)
	{
    osMutexWait(myMutex01Handle, osWaitForever);
		vx = speed * cos(M_PI / 180 * mpu.psi);
		vy = speed * sin(M_PI / 180 * mpu.psi);
		osMutexRelease(myMutex01Handle);
    real_x += dt * vx;
    real_y +=  dt * vy;

		predict(&kf, dt);

		if(counter % 30 == 0){
		  update(&kf, real_x, real_y, meas_variance);
    }
	      // update(&kf, dataCam.px, dataCam.py, meas_variance);
		counter++;

		// printf("Real: %3.3f, %3.3f || Estimated: %3.3f, %3.3f || V: %3.3f, %3.3f || Psi: %3.3f \r\n",
		// real_x, real_y, kf.x[0], kf.y[0], vx, vy, mpu.psi);
		osDelay(dt * 1000);
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
  
  float in[2] = {st_saturation_limits[1], st_saturation_limits[0]}; // min, max delta values
  float out[2] = {10, 90}; // min, max percentage values
  float slope = (float)((out[1] - out[0]) / (in[1] - in[0]));

  uint8_t last_steer = 0;

  servoValues.percentage = 50;
  setPwmS(&servoValues);

  canTaskHandle = osThreadNew(canTask, NULL, &canTaskHandle_attributes);

	for(;;){
    osMutexWait(myMutex01Handle, osWaitForever);
     servoValues.percentage = (int) ( (out[0] + (slope * (stanley.delta - in[0]))));
		osMutexRelease(myMutex01Handle);
     if(servoValues.percentage != last_steer){
    	 setPwmS(&servoValues);

     }
     last_steer = servoValues.percentage;
    osDelay(100);
	}
}

void stanleyTask(void *argument){
  float st_k = 1; // Gain
  float st_k_soft = 1; // Soft gain
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
    osMutexWait(myMutex01Handle, osWaitForever);
    vehicle_pos.x = kf.x[0];
    vehicle_pos.y = kf.y[0];

    if(wp_buf.size > 1){
      p1.x = wp_buf.wp_buf[wp_buf.from].x;
      p1.y = wp_buf.wp_buf[wp_buf.from].y;
      p2.x = wp_buf.wp_buf[wp_buf.to].x;
      p2.y = wp_buf.wp_buf[wp_buf.to].y;
    }
    psi = mpu.psi * M_PI / 180;
    vel = 1;
    calculateCrosstrackError(&stanley, &vehicle_pos, &p1, &p2);
    setYawAngle(&stanley, psi);
    calculateSteering(&stanley, 1, precision);
		osMutexRelease(myMutex01Handle);

		// osMutexWait(myMutex01Handle, osWaitForever);
    printf("P{%3.2f,%3.2f,%3.2f} WP{%3.1f,%3.1f}, Delta{%3.1f},"
    		"E{%3.1f}, S{%1.2f}\r\n",
      kf.x[0], kf.y[0], mpu.psi,
      wp_buf.wp_buf[wp_buf.to].x, wp_buf.wp_buf[wp_buf.to].y,
      (stanley.delta * 180 / M_PI), stanley.e_a, speed);
		// osMutexRelease(myMutex01Handle);
    osDelay(100);
	}
}

void wirelessTask(void *argument){
	uint64_t RxpipeAddrs = 0x11223344AA; //Address of sender

	//osMutexWait(myMutex01Handle, osWaitForever); //Setting up Radio
	mySetupNRF24(nrf_CSN_PORT3_2, nrf_CSN_PIN3_2, nrf_CE_PIN3_2,
			  hspi3,huart3,52, RxpipeAddrs, 1);
	//osMutexRelease(myMutex01Handle);

	int maxX = 306; //Max and min values
	int minX = 0;
	int maxY = 170;
	int minY = 0;
	int maxA = 360;
	int minA = 0;

	// stanleyTaskHandle = osThreadNew(stanleyTask, NULL, &stanleyTaskHandle_attributes);

	for(;;){
		myReadData(myRxData);
		dataCam.x = (uint16_t)myRxData[0] << 8 | myRxData[1];
		dataCam.y = (uint16_t)myRxData[2] << 8 | myRxData[3];
		dataCam.theta = (uint16_t)myRxData[4] << 8 | myRxData[5];

		dataCam.x = dataCam.x < minX ? minX : dataCam.x;
		dataCam.x = dataCam.x > maxX ? maxX : dataCam.x;
		dataCam.y = dataCam.y < minY ? minY : dataCam.y;
		dataCam.y = dataCam.y > maxY ? maxY : dataCam.y;
		dataCam.theta = dataCam.theta < minA ? minA : dataCam.theta;
		dataCam.theta = dataCam.theta > maxA ? maxA : dataCam.theta;

		dataCam.px = dataCam.x / 100.0;
		dataCam.py = dataCam.y / 100.0;
		dataCam.psi = dataCam.theta;
		if(dataCam.psi > 180)
			dataCam.psi -= 360;
//		for(int i=0;i<6;i++){
//			printf("%x ",myRxData[i]);
//		}
//		printf("%x%x %x%x %x%x",myRxData[0],myRxData[1],myRxData[2],myRxData[3],myRxData[4],myRxData[5]);
//		printf("\r\n");
//		printf("X:%d Y:%d Angle:%d\r\n",dataCam.px,dataCam.py,dataCam.psi);
//		printf("\r\n");
		osDelay(100);
	}
}

void wpTask(void *argument) {
  init_waypoint_buffer(&wp_buf);
//  add_wp(&wp_buf, 0, 0);
//  add_wp(&wp_buf, 1, 0);
//  add_wp(&wp_buf, 1, 1);
//  add_wp(&wp_buf, 0, 1);
//  add_wp(&wp_buf, 0, 0);
  float wx = initPose[0];
  float wy = initPose[1];
  float tok = 0;
  int samples = 10;
  float dWp = M_PI * 4.0 / samples; // 2 sinusoidal periods / samples
//  for(int i = 0 ; i < samples ; i++){
//	  tok += dWp;
//	  wx = 4* sin(tok);
//	  wy = 3 * cos(tok / 2);
//	  add_wp(&wp_buf, wx, wy);
//  }
//  add_wp(&wp_buf, 0, 0.2);
//  add_wp(&wp_buf, 1, 0.2);
//  add_wp(&wp_buf, 1, 0.8);
//  add_wp(&wp_buf, 0, 1);
//  add_wp(&wp_buf, 0, 0);

  add_wp(&wp_buf,1.5 * 0, 0 * 0.7);
  add_wp(&wp_buf,1.5 * 0.68, -1 * 0.7);
  add_wp(&wp_buf,1.5 * 1, 0 * 0.7);
  add_wp(&wp_buf,1.5 * 0.68, 1 * 0.7);
  add_wp(&wp_buf,1.5 * -0.68, -1 * 0.7);
  add_wp(&wp_buf,1.5 * -1, 0.1 * 0.7);
  add_wp(&wp_buf,1.5 * -0.68, 1 * 0.7);
  add_wp(&wp_buf,1.5 * 0, 0 * 0.7);


  // printf("Waypoints: \r\n");
  // for(int i = 0 ; i < wp_buf.size ; i++){
	//   printf("%d: {%3.3f, %3.3f}\r\n", i, wp_buf.wp_buf[i].x, wp_buf.wp_buf[i].y);
  // }

  float dt = 0.1;
  clean(&wp_buf);
	for(;;)
	{
	if(reached)
		printf("REACHED!");

  osMutexWait(myMutex01Handle, osWaitForever);
  if(stanley.e_a < 0.2 && stanley.e_a > -0.2 && stanley.e_a != 0){
    if(wp_buf.to == wp_buf.size - 1)
    reached = true;
    to_next(&wp_buf);
    printf("STANLEY.E_A %3.3f\r\n", stanley.e_a);
  }
  osMutexRelease(myMutex01Handle);
	osDelay(dt * 1000);
	}
}

// m = [
// [0.588, 1.902],
// [0.951, 1.618],
// [-0.951, 1.618],
// [-0.588, 1.902],
// [0.000, 2.000],
// ];
// plot(m(:,1), m(:,2));

/* USER CODE END Application */

