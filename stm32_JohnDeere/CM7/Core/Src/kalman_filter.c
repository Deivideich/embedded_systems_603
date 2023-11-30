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
#include "myprintf.h"

void initKalman(struct KF * kf, 
    float initial_x, float initial_y,
    float initial_vx, float initial_vy,
    float accel_var_x, float accel_var_y) {
    float I[2][2] = {{1.0,0.0},{0.0,1.0}};
    kf->x[0] = initial_x;
    kf->x[1] = initial_vx;
    kf->y[0] = initial_y;
    kf->y[1] = initial_vy;
    kf->accel_variance_x = accel_var_x;
    kf->accel_variance_y = accel_var_y;

    for(int i = 0 ; i < 2 ; i++){
        for(int j = 0 ; j < 2 ; j++){
            kf->Px[i][j] = I[i][j];
            kf->Py[i][j] = I[i][j];
        }
    }
}

void predict(struct KF * kf, float dt) {
    float F[2][2] = {{1, dt}, {0, 1}};
    float G[2] = {0.5 * dt * dt, dt};

    // x = F * x
    float x_new[2];
    float y_new[2];
    dotV(kf->x, F, x_new);
    dotV(kf->y, F, y_new);
    copy(x_new, kf->x);
    copy(y_new, kf->y);

    // P = F * P * Ft + G * Gt * accel_variance
    float temp1_x[2][2];
    float temp1_y[2][2];
    float temp2_x[2][2];
    float temp2_y[2][2];
    float FPx[2][2];
    float FPy[2][2];

    // FP = F * P
    dot(F, kf->Px, FPx);
    dot(F, kf->Py, FPy);

    // P = FP * Ft
    float Ft[2][2];
    trans(F, Ft);
    dot(FPx, Ft, kf->Px);
    dot(FPy, Ft, kf->Py);

    // G * Gt
    float GGt = G[0] * G[0] + G[1] * G[1];

    // P = P + GGt * accel_variance
    kf->Px[0][0] += GGt * kf->accel_variance_x;
    kf->Px[1][1] += GGt * kf->accel_variance_x;
    kf->Py[0][0] += GGt * kf->accel_variance_y;
    kf->Py[1][1] += GGt * kf->accel_variance_y;
}

void update(struct KF * kf, float meas_value_x, float meas_value_y, float meas_variance) {
    float H[2] = {1, 0};

    // Y = z - H * x
    float Yx = meas_value_x - H[0] * kf->x[0] - H[1] * kf->x[1];
    float Yy = meas_value_y - H[0] * kf->y[0] - H[1] * kf->y[1];

    // S = H * P * Ht + R
    float Hpx[2];
    float Hpy[2];
    dotV(H, kf->Px, Hpx);
    dotV(H, kf->Py, Hpy);
    
    float Sx = Hpx[0]*H[0] + Hpx[1]*H[1] + meas_variance;
    float Sy = Hpy[0]*H[0] + Hpy[1]*H[1] + meas_variance;

    // K = P * Ht * S^-1
    float Kx[2];
    float Ky[2];
    float PHtx[2];
    float PHty[2];
    dotV(H, kf->Px, PHtx);
    dotV(H, kf->Py, PHty);
    dotS(1/Sx, PHtx, Kx);
    dotS(1/Sy, PHty, Ky);

    // x = x + K * y
    kf->x[0] = kf->x[0] + Kx[0] * Yx;
    kf->x[1] = kf->x[1] + Kx[1] * Yx;
    kf->y[0] = kf->y[0] + Ky[0] * Yy;
    kf->y[1] = kf->y[1] + Ky[1] * Yy;

    // P = (I - K * H) * P
    float I[2][2] = {{1 , 0} , {0 , 1}};
    float KHx = Kx[0] * H[0] + Kx[1] * H[1];
    float KHy = Ky[0] * H[0] + Ky[1] * H[1];

    float IKHx[2][2];
    float IKHy[2][2];
    sumS(KHx, I, IKHx);
    sumS(KHy, I, IKHy);

    float Px_t[2][2];
    float Py_t[2][2];

    dot(IKHx, kf->Px, Px_t);
    dot(IKHy, kf->Py, Py_t);
    copy(Px_t, kf->Px);
    copy(Py_t, kf->Py);
    sat(kf->Px);
    sat(kf->Py);
}

void dot(float a[2][2], float b[2][2], float r[2][2]){
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            r[i][j] = 0.0;
            for (int k = 0; k < 2; ++k) {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

void dotV(float a[2], float b[2][2], float r[2]){
    for (int i = 0; i < 2; ++i) {
        r[i] = 0;
        for (int j = 0; j < 2; ++j) {
            r[i] += b[i][j] * a[j];
        }
    }
}

void dotS(float a, float b[2], float r[2]){
    for (int i = 0; i < 2; ++i) {
        r[i] = b[i]*a;
    }
}

void copy(float a[2][2], float r[2][2]){
    for(int i = 0 ; i < 2 ; i++){
        for(int j = 0 ; j < 2 ; j++){
            r[i][j] = a[i][j];
        }
    }
}

void sumS(float a, float b[2][2],  float r[2][2]){
    for(int i = 0 ; i < 2 ; i++){
        for(int j = 0 ; j < 2 ; j++){
            r[i][j] = b[i][j] + a;
        }
    }
}

void trans(float a[2][2], float r[2][2]){
    for(int i = 0 ; i < 2 ; i++){
        for(int j = 0 ; j < 2 ; j++){
            r[i][j] = a[j][i];
        }
    }
}

void sat(float r[2][2]){
    float sat[2] = {10000.0, -10000.0};
    for(int i = 0 ; i < 2 ; i++){
        for(int j = 0 ; j < 2 ; j++){
            r[i][j] = r[i][j] < sat[1] ? sat[1] : r[i][j];
            r[i][j] = r[i][j] > sat[0] ? sat[0] : r[i][j];
        }
    }
}

#include <stdio.h>