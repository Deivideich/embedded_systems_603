/*
 * esc.c
 *
 *  Created on: 3 Nov 2023
 *      Author: demia
 */
#include "tim.h"


void startSetup(TIM_HandleTypeDef htimN){
  double pulseWidth = 0.0015;
  double ccr = 0;

  double i = 0;

  do{
	  ccr = (pulseWidth * htimN.Init.Period) / 0.02;
	  htimN.Instance->CCR1 = ccr;
	  HAL_Delay(100);

//	  printf("pulseWidth = %f \r\n", pulseWidth);
	  i += 0.000001;

	  pulseWidth += i;

  }while(pulseWidth < 0.002);

  pulseWidth = 0.002;
  ccr = (pulseWidth * htimN.Init.Period) / 0.02;
}
