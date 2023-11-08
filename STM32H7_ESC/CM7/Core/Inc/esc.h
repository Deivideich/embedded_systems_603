/*
 * esc.h
 *
 *  Created on: 3 Nov 2023
 *      Author: demia
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

#include "stm32h7xx_hal.h"

#define MY_SATURATE(value, max) ( (value) > (max) ? (max) : (value) )

struct escValues{
	TIM_HandleTypeDef htimN;
	unsigned int minPulseWidth;
	unsigned int maxPulseWidth;
	unsigned int pwmPeriod;
	unsigned int resolution;
	unsigned int percentage;

	double pulseWidth;
};

void setPwm(TIM_HandleTypeDef htimN, unsigned int minPulseWidth,
		unsigned int maxPulseWidth, unsigned int pwmPeriod, unsigned int resolution, unsigned int percentage);

void setPwmS(struct escValues *escValues);

#endif /* INC_ESC_H_ */
