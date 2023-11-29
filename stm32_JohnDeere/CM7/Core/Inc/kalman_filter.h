/* 
 * ----------------------------------------------------------------------------
 * @file: kalman_filter.h
 * @date: November 28th, 2023
 * @author: Max Pacheco
 * @email: maxprpxam@icloud.com
 * 
 * @brief: Kalman Filter class
 * Based on CppMonk Youtube Channel's Python implementation
 * ----------------------------------------------------------------------------
 * */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "stdint.h"
#include "math.h"
#include "stdio.h"

struct KF{
    float x[2];
    float y[2];
    float accel_variance_x;
    float accel_variance_y;
    float Px[2][2];
    float Py[2][2];
};

void initKalman(struct KF * kf, float initial_x, float initial_y,
                float initial_vx, float initial_vy,
                float accel_var_x, float accel_var_y);
void predict(struct KF * kf, float dt);
void update(struct KF * kf, float meas_value_x, float meas_value_y, float meas_variance);

#endif /* KALMAN_FILTER_H_ */