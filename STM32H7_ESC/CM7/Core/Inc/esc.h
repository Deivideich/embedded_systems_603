/*
 * esc.h
 *
 *  Created on: 3 Nov 2023
 *      Author: demia
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

#define MY_SATURATE(value, max) ( (value) > (max) ? (max) : (value) )

#include "stm32h7xx_hal.h"

void setPwm(TIM_HandleTypeDef htimN, unsigned int minPulseWidth,
		unsigned int maxPulseWidth, unsigned int resolution, unsigned int percentage);

#endif /* INC_ESC_H_ */
