/*
 * esc.c
 *
 *  Created on: 3 Nov 2023
 *      Author: demia
 */
#include <stdlib.h>
#include "tim.h"
#include "esc.h"




//@brief
//@param minPulseWdith, maxPulseWidth and pwmPeriod in micro seconds.
//@param Resolution and percentage in integer.
//@Note: The pwm is set in percentage of the resolution assigned. If resolution assigned 100. Then
//percentage = 1 will create a pulse width of minPulseWidth*e-06 and 100 a pulse width of maxPulseWidth*e-06
void setPwm(TIM_HandleTypeDef htimN, unsigned int minPulseWidth,
		unsigned int maxPulseWidth, unsigned int pwmPeriod, unsigned int resolution, unsigned int percentage){

	double pulseWidth;
	double ccr;
	double pwmPeriodInMicroSeconds; //Actually in seconds

	minPulseWidth = MY_SATURATE(minPulseWidth,pwmPeriod);
	maxPulseWidth = MY_SATURATE(maxPulseWidth,pwmPeriod);
	percentage = MY_SATURATE(percentage, resolution);

	pulseWidth =  ( ( (maxPulseWidth - minPulseWidth)/resolution ) * percentage ) + minPulseWidth;

	pulseWidth = pulseWidth/1e6;
	pwmPeriodInMicroSeconds = pwmPeriod/1e6;

	ccr = (pulseWidth * htimN.Init.Period) / pwmPeriodInMicroSeconds;
	htimN.Instance->CCR1 = ccr;


}

void setPwmS(struct escValues *escValues){
	double ccr;
	double pwmPeriodInSeconds; //Fix from setPwmS

	escValues->minPulseWidth = MY_SATURATE(escValues->minPulseWidth,escValues->pwmPeriod);
	escValues->maxPulseWidth = MY_SATURATE(escValues->maxPulseWidth,escValues->pwmPeriod);
	escValues->percentage = MY_SATURATE(escValues->percentage, escValues->resolution);

	escValues->pulseWidth =  ( ( (escValues->maxPulseWidth - escValues->minPulseWidth)/escValues->resolution ) * escValues->percentage ) + escValues->minPulseWidth;

	escValues->pulseWidth = escValues->pulseWidth/1e6;
	pwmPeriodInSeconds = escValues->pwmPeriod/1e6;

	ccr = (escValues->pulseWidth * escValues->htimN.Init.Period) / pwmPeriodInSeconds;
	escValues->htimN.Instance->CCR1 = ccr;
}
