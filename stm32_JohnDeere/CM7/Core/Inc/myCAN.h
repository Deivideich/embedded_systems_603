/*
 * myCAN.h
 *
 *  Created on: Nov 29, 2023
 *      Author: demia
 */

#ifndef INC_MYCAN_H_
#define INC_MYCAN_H_

#include "stdint.h"
#include "fdcan.h"

union BytesFloat {
	float floatValue;
	uint8_t byteValue[sizeof(float)];
};

uint8_t readSpeed(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *pRxHeader,
		union BytesFloat bf, uint8_t *pRxData, float *speed);

#endif /* INC_MYCAN_H_ */
