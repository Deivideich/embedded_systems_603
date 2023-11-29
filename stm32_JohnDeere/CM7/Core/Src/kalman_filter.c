/* 
 * ----------------------------------------------------------------------------
 * @file: kalman_filter.c
 * @date: November 28th, 2023
 * @author: Max Pacheco
 * @email: maxprpxam@icloud.com
 * 
 * @brief: Kalman Filter class
 * Based on CppMonk Youtube Channel's Python implementation
 * ----------------------------------------------------------------------------
 * */

#include "kalman_filter.h"

void initKalman(struct KF * kf, 
    float initial_x, float initial_y,
    float initial_vx, float initial_vy,
    float accel_var_x, float accel_var_y) {
    float I[2][2] = {{1,0},{0,1}};
    kf->x[0] = initial_x;
    kf->x[1] = initial_vx;
    kf->y[0] = initial_y;
    kf->y[1] = initial_vy;
    kf->accel_variance_x = accel_var_x;
    kf->accel_variance_y = accel_var_y;

    for(int i = 0 ; i < 2 ; i++){
        for(int j = 0 ; i < 2 ; j++){
            kf->Px[i][j] = I[i][j];
            kf->Py[i][j] = I[i][j];
        }
    }
}

void predict(struct KF * kf, float dt) {
    float F[2][2] = {{1, dt}, {0, 1}};
    float G[2][1] = {0.5 * dt * dt, dt};

    // x = F * x
    kf->x[0] = F[0][0] * kf->x[0] + F[0][1] * kf->x[1];
    kf->x[1] = F[1][0] * kf->x[0] + F[1][1] * kf->x[1];
    // y = F * y
    kf->y[0] = F[0][0] * kf->y[0] + F[0][1] * kf->y[1];
    kf->y[1] = F[1][0] * kf->y[0] + F[1][1] * kf->y[1];

    // P = F * P * Ft + G * Gt * accel_variance
    float temp_x[2][2];
    float temp_y[2][2];

    // F * P
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            temp_x[i][j] = 0.0;
            temp_y[i][j] = 0.0;
            for (int k = 0; k < 2; ++k) {
                temp_x[i][j] += F[i][k] * kf->Px[k][j];
                temp_y[i][j] += F[i][k] * kf->Py[k][j];
            }
        }
    }

    // P = temp * Ft
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            kf->Px[i][j] = 0.0;
            kf->Py[i][j] = 0.0;
            for (int k = 0; k < 2; ++k) {
                kf->Px[i][j] += temp_x[i][k] * F[j][k];
                kf->Py[i][j] += temp_y[i][k] * F[j][k];
            }
        }
    }

    // G * Gt
    float GGt = G[0][0] * G[0][0] + G[1][0] * G[1][0];

    // P = P + GGt * accel_variance
    kf->Px[0][0] += GGt * kf->accel_variance_x;
    kf->Px[1][1] += GGt * kf->accel_variance_x;
    kf->Py[0][0] += GGt * kf->accel_variance_y;
    kf->Py[1][1] += GGt * kf->accel_variance_y;
}

void update(struct KF * kf, float meas_value_x, float meas_value_y, float meas_variance) {
    float H[1][2] = {{1, 0}};

    // Y = z - H * x
    float Yx = meas_value_x - H[0][0] * kf->x[0] - H[0][1] * kf->x[1];
    float Yy = meas_value_y - H[0][0] * kf->y[0] - H[0][1] * kf->y[1];

    // S = H * P * Ht + R
    float Sx = H[0][0] * kf->Px[0][0] * H[0][0] + H[0][1] * kf->Px[1][1] * H[0][1] + meas_variance;
    float Sy = H[0][0] * kf->Py[0][0] * H[0][0] + H[0][1] * kf->Py[1][1] * H[0][1] + meas_variance;

    // K = P * Ht * S^-1
    float Kx[2] = {kf->Px[0][0] * H[0][0] / Sx, kf->Px[1][1] * H[0][1] / Sx};
    float Ky[2] = {kf->Py[0][0] * H[0][0] / Sy, kf->Py[1][1] * H[0][1] / Sy};

    // x = x + K * y
    kf->x[0] = kf->x[0] + Kx[0] * Yx;
    kf->x[1] = kf->x[1] + Kx[1] * Yx;
    kf->y[0] = kf->y[0] + Ky[0] * Yy;
    kf->y[1] = kf->y[1] + Ky[1] * Yy;

    // P = (I - K * H) * P
    kf->Px[0][0] = (1 - Kx[0] * H[0][0]) * kf->Px[0][0];
    kf->Px[1][1] = (1 - Kx[1] * H[0][1]) * kf->Px[1][1];
    kf->Py[0][0] = (1 - Ky[0] * H[0][0]) * kf->Py[0][0];
    kf->Py[1][1] = (1 - Ky[1] * H[0][1]) * kf->Py[1][1];
}

#include <stdio.h>