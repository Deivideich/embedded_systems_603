/*
 * myCAN.c
 *
 *  Created on: Nov 29, 2023
 *      Author: demia
 */

#include "myCAN.h"

void readSpeed(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *pRxHeader,
		union BytesFloat bf, uint8_t *pRxData, uint8_t *m, float *sp){
	uint8_t m2;
	m2 = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, pRxHeader, pRxData);
	*m = m2;
	for (size_t i = 0; i < sizeof(float); i++) {
		uint8_t receivedByte = pRxData[i];
		bf.byteValue[i] = receivedByte;
	}

	*sp = bf.floatValue;

	// return m;
}
