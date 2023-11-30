/* 
 * ----------------------------------------------------------------------------
 * @file: pi.h
 * @date: November 30th, 2023
 * @author: Max Pacheco
 * @email: maxprpxam@icloud.com
 * @author: Deivideich!
 * 
 * @brief: PI controller class
 * 
 * ----------------------------------------------------------------------------
 * */

#ifndef PI_H_
#define PI_H_

#include "stdint.h"
#include "math.h"
#include "stdio.h"

struct PI{
	float sample_time;
	float error;
	float prev_error;
	float k_p;
	float k_i;
	float k_d;
	float U_MIN;
	float U_MAX;
	float u;
	float chi1_d;
};

void initPI(struct PI * pi, float sample_time, float k_p, float k_i, float k_d, float u_max, float u_min);
void updateReferences(struct PI * pi, float chi1_d);
void calculateManipulation(struct PI * pi, float chi1);
void saturateManipulation(struct PI * pi, float chi1);

#endif /* PI_H_ */