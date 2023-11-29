/*
 * myCAN.c
 *
 *  Created on: Nov 29, 2023
 *      Author: demia
 */

#include "myCAN.h"

uint8_t readSpeed(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *pRxHeader,
		union BytesFloat bf, uint8_t *pRxData, float *speed){
	uint8_t m;
	m = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, pRxHeader, pRxData);
	for (size_t i = 0; i < sizeof(float); i++) {
		uint8_t receivedByte = pRxData[i];
		bf.byteValue[i] = receivedByte;
	}

	*speed = bf.floatValue;

	return m;
}
