/** ----------------------------------------------------------------------------
 * @file: stanley_controller.h
 * @date: November 24th, 2023
 * @author: Max Pacheco
 * @email: maxprpxam@icloud.com
 * 
 * @brief: Stanley Controller class
 * Obtained from vanttec_controllers repository
 * -----------------------------------------------------------------------------
 * */

#ifndef STANLEY_CONTROLLER_H_
#define STANLEY_CONTROLLER_H_

#include "stdint.h"
#include "math.h"
#include "stdio.h"

struct Point{
    float x;
    float y;
};

struct Stanley{
    float sat[2];     // {max, min} steering
    float delta;            // Desired steering
    float psi;              // Current heading
    float k;                // Controller gain
    float k_soft;           // Soft gain
    float e_a;               // Along-track error
    float e_c;               // Crosstrack error
    float vel;              // velocity vector norm
    float ak;               // path angle
};

void initStanley(struct Stanley * stanley, float *delta_sat, float k, float k_soft);
void calculateCrosstrackError(struct Stanley * stanley, struct Point * vehicle_pos, struct Point * p1, struct Point * p2);
void setYawAngle(struct Stanley * stanley, float psi);
void calculateSteering(struct Stanley * stanley, float vel, uint8_t precision);

#endif /* STANLEY_CONTROLLER_H_ */