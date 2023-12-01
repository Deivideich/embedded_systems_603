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

<<<<<<< Updated upstream
=======
osThreadId_t wirelessTaskHandle;
const osThreadAttr_t wirelessTaskHandle_attributes = {
		.name = "wirelessTask",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityHigh,
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

>>>>>>> Stashed changes
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
/* Definitions for myMutex02 */
osMutexId_t myMutex02Handle;
const osMutexAttr_t myMutex02_attributes = {
  .name = "myMutex02"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void blinkGreenTask(void *argument);
void imuTask(void *argument);
void escTask(void *argument);
void servoTask(void *argument);
void stanleyTask(void *argument);
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

  /* creation of myMutex02 */
  myMutex02Handle = osMutexNew(&myMutex02_attributes);

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
<<<<<<< Updated upstream
  escTaskHandle = osThreadNew(escTask, NULL, &escTaskHandle_attributes);
//   servoTaskHandle = osThreadNew(servoTask, NULL, &servoTaskHandle_attributes);
  // escTaskHandle = osThreadNew(escTask, NULL, &escTaskHandle_attributes);
=======

  servoTaskHandle = osThreadNew(servoTask, NULL, &servoTaskHandle_attributes); //Starting tasks
  escTaskHandle = osThreadNew(escTask, NULL, &escTaskHandle_attributes);
  radioTaskHandle = osThreadNew(radioTask, NULL, &radioTaskHandle_attributes);
  wpTaskHandle = osThreadNew(wpTask, NULL, &wpTaskHandle_attributes);

    //imuTaskHandle = osThreadNew(imuTask, NULL, &imuTaskHandle_attributes);
  // servoTaskHandle = osThreadNew(servoTask, NULL, &servoTaskHandle_attributes);
//    wirelessTaskHandle = osThreadNew(wirelessTask, NULL, &wirelessTaskHandle_attributes);
   // canTaskHandle = osThreadNew(canTask, NULL, &canTaskHandle_attributes);
  //
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
=======
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
		vx = speed * cos(M_PI / 180 * mpu.pose[2]);
		vy = speed * sin(M_PI / 180 * mpu.pose[2]);
	    real_x += dt * vx;
	    real_y +=  dt * vy;

		predict(&kf, dt);

		if(counter % 10 == 0){

//		  update(&kf, real_x, real_y, meas_variance);
	      update(&kf, dataCam.px, dataCam.py, meas_variance);
		}
		counter++;

		// printf("Real: %3.3f, %3.3f || Estimated: %3.3f, %3.3f || V: %3.3f, %3.3f || Psi: %3.3f \r\n",
		// real_x, real_y, kf.x[0], kf.y[0], vx, vy, mpu.pose[2]);
		osDelay(dt * 1000);
	}
}

void wpTask(void *argument) {
  init_waypoint_buffer(&wp_buf);
//  add_wp(&wp_buf, 0, 0);
//  add_wp(&wp_buf, 1, 0);
//  add_wp(&wp_buf, 1, 1);
//  add_wp(&wp_buf, 0, 1);
//  add_wp(&wp_buf, 0, 0);
  add_wp(&wp_buf, 0.1, 0.85);
  add_wp(&wp_buf, 2.5, 0.85);
  float dt = 0.1;
	for(;;)
	{
    if(stanley.e_a < 0.2 && stanley.e_a > 0.0)
      to_next(&wp_buf);
	osDelay(dt * 1000);
	}
}

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
    stanleyTaskHandle = osThreadNew(stanleyTask, NULL, &stanleyTaskHandle_attributes);
    float dt = 0.05;
	for(;;)
	{
		updateData(&mpu, 0.1, dt); //Printing with func from header file
=======
	  int counter = 1;

	wirelessTaskHandle = osThreadNew(wirelessTask, NULL, &wirelessTaskHandle_attributes);

    float dt = 0.1;
	for(;;)
	{
		updateData(&mpu, dt, speed); //Printing with func from header file
		if(counter % 30 == 0){
			pose[2] = dataCam.psi;
			setPose(&mpu, pose);
		}
		counter++;

>>>>>>> Stashed changes
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
    // osMutexWait(myMutex01Handle, osWaitForever);
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
<<<<<<< Updated upstream
	  setPwmS(&escValues);
	  escValues.percentage=(unsigned int)i;
	  HAL_Delay(10);
	  i=i-dt;
=======
	osMutexWait(myMutex01Handle, osWaitForever);
	setPwmS(&escValues);
	osMutexRelease(myMutex01Handle);

	escValues.percentage=(unsigned int)i;
	HAL_Delay(10);
	i=i-dt;
>>>>>>> Stashed changes

  }while(i > 50);

  minPulseWidthEsc = 1500;
  maxPulseWidthEsc = 2000;
  escValues.maxPulseWidth = maxPulseWidthEsc;
  escValues.minPulseWidth = minPulseWidthEsc;

  imuTaskHandle = osThreadNew(imuTask, NULL, &imuTaskHandle_attributes);

<<<<<<< Updated upstream
	escValues.percentage = 30;
	setPwmS(&escValues);
	for(;;){
    osDelay(100);
=======
  //escValues.percentage = 0;
  //setPwmS(&escValues);

  updateReferences(&pi, 0.5);
  saturateManipulation(&pi, speed);
  //escValues.percentage = (unsigned int)pi.u;
//  escValues.percentage = 30;
  escValues.percentage = 0;
//    osMutexRelease(myMutex01Handle);
	setPwmS(&escValues);
//	osMutexWait(myMutex01Handle, osWaitForever);
  HAL_Delay(10);

	for(;;){

		if(wp_buf.to == wp_buf.size - 1 && (stanley.e_a < 0.1 || stanley.e_a > -0.1)){
		  escValues.percentage = 0;
//		    osMutexWait(myMutex01Handle, osWaitForever);
//		  setPwmS(&escValues);
//    		osMutexRelease(myMutex01Handle);
		} else {
		  saturateManipulation(&pi, speed);
//		  escValues.percentage = (unsigned int)pi.u;
		  //escValues.percentage = 80;
//			osMutexWait(myMutex01Handle, osWaitForever);
//		  setPwmS(&escValues);
//    		osMutexRelease(myMutex01Handle);
		  ;
		}
		osDelay(1000 * dtTask);
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

  float in[2] = {stanley.sat[1], stanley.sat[0]}; // min, max delta values
  float out[2] = {90, 0}; // min, max percentage values
  float slope = (float)(out[1] - out[0]) / (in[1] - in[0]);
=======
  
  float in[2] = {st_saturation_limits[1], st_saturation_limits[0]}; // min, max delta values
  float out[2] = {10, 90}; // min, max percentage values
  float slope = (float)((out[1] - out[0]) / (in[1] - in[0]));
>>>>>>> Stashed changes

  uint8_t last_steer = 0;

  servoValues.percentage = 50;
  setPwmS(&servoValues);

<<<<<<< Updated upstream
	for(;;){
     servoValues.percentage = (int) ( (out[0] + (slope * (stanley.delta - in[0]))));
//     osMutexWait(myMutex01Handle, osWaitForever);
	 printf("Y {%u}",servoValues.percentage);
     if(servoValues.percentage != last_steer){
       setPwmS(&servoValues);
	   printf("servo {%u},last {%u}",servoValues.percentage, last_steer);
=======
  canTaskHandle = osThreadNew(canTask, NULL, &canTaskHandle_attributes);
  float offset_delta = 21.4;

	for(;;){
     servoValues.percentage = (int) ( (out[0] + (slope * ((stanley.delta+offset_delta) - in[0]))));
     if(servoValues.percentage != last_steer){
   		setPwmS(&servoValues);
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
    osMutexWait(myMutex01Handle, osWaitForever);
    printf(" Pose XYZ:");
	for(int i = 0; i<3;i++){
	   printf("{%05.1f}",mpu.pose[i]);
	}
	printf(" Delta: {%05.1f}",stanley.delta * 180 / M_PI);
	printf("\r\n");
    osMutexRelease(myMutex01Handle);
    osDelay(50);
=======

    printf("Pose{%3.1f,%3.1f,%3.1f}, WP{%3.1f,%3.1f}, Delta{%3.1f},"
    		"E{%3.1f}, S{%1.2f}, PI{sp:%3.3f,u:%3.3f} Cam{%3.3f,%3.3f,%3.3f}\r\n",
      kf.x[0], kf.y[0], mpu.pose[2],
      wp_buf.wp_buf[wp_buf.to].x, wp_buf.wp_buf[wp_buf.to].y,
      (stanley.delta * 180 / M_PI), stanley.e_a, speed, pi.chi1_d, pi.u,
	  dataCam.px,dataCam.py,dataCam.psi);
    osDelay(100);
	}
}

void wirelessTask(void *argument){
	uint64_t RxpipeAddrs = 0x11223344AA; //Address of sender
	uint8_t m;
//	osMutexWait(myMutex02Handle, osWaitForever); //Setting up Radio
//	mySetupNRF24(nrf_CSN_PORT, nrf_CSN_PIN, nrf_CE_PIN,
//			  hspi2,huart3,52, RxpipeAddrs, 1);
	mySetupNRF24(nrf_CSN_PORT3_2, nrf_CSN_PIN3_2, nrf_CE_PIN3_2,
				  hspi3,huart3,52, RxpipeAddrs, 1);
//	osMutexRelease(myMutex02Handle);

	int maxX = 306; //Max and min values
	int minX = 0;
	int maxY = 170;
	int minY = 0;
	int maxA = 360;
	int minA = 0;

	stanleyTaskHandle = osThreadNew(stanleyTask, NULL, &stanleyTaskHandle_attributes);

	for(;;){
		m = myReadData(myRxData);
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
//		mpu.pose[2] = dataCam.psi * M_PI / 180;


//		printf("S: %d |",m);
//		printf("%x%x %x%x %x%x",myRxData2[0],myRxData2[1],myRxData2[2],myRxData2[3],myRxData2[4],myRxData2[5]);
//		printf("%x%x %x%x %x%x",myRxData[0],myRxData[1],myRxData[2],myRxData[3],myRxData[4],myRxData[5]);
//		printf("\r\n");
		printf("X:%3.3f Y:%3.3f Angle:%3.3f\r\n",dataCam.px,dataCam.py,dataCam.psi);
		printf("\r\n");
		osDelay(100);
	}
}

void canTask(void *argument){
	FDCAN_RxHeaderTypeDef RxHeader;
	union BytesFloat bf;
	uint8_t RxData[8];
	//float speed=1.3;
	uint8_t m;
	float loc_speed;

	printf("Starting CAN...\r\n");
	for(;;){
		// osMutexWait(myMutex01Handle, osWaitForever); //Setting up Radio
		readSpeed(&hfdcan1, &RxHeader, bf, RxData, &m, &loc_speed);
		speed = loc_speed / 10;
		if(speed<0){
			speed=speed*-1;
		}
		// osMutexRelease(myMutex01Handle);
//		printf("S:%d {",m);
//		for(int i=0;i<8;i++){
//			printf("%x",RxData[i]);
//		}
////		speed = (float)loc_sp;
//		printf("} %3.3f, CAN: %3.3f \r\n",speed, speed);
		osDelay(100);
>>>>>>> Stashed changes
	}
}
/* USER CODE END Application */

