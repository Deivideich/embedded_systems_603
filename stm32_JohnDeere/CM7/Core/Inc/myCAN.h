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

void readSpeed(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *pRxHeader,
		union BytesFloat bf, uint8_t *pRxData, uint8_t *m, float *sp);

#endif /* INC_MYCAN_H_ */
