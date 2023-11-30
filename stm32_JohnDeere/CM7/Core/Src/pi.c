/* 
 * ----------------------------------------------------------------------------
 * @file: pi.c
 * @date: November 30th, 2023
 * @author: Max Pacheco
 * @email: maxprpxam@icloud.com
 * @author: Deivideich!
 * 
 * @brief: PI controller class
 * 
 * ----------------------------------------------------------------------------
 * */

#include "pi.h"
#include "myprintf.h"

void initPI(struct PI * pi, float sample_time, float k_p, float k_i, float k_d, float u_max, float u_min){
  pi->sample_time = sample_time;
  pi->k_p = k_p;
  pi->k_i = k_i;
  pi->k_d = k_d;

  pi->error = 0;
  pi->prev_error = 0;
  pi->chi1_d = 0;
  pi->u = 0;

  pi->U_MAX = u_max;
  pi->U_MIN = u_min;
}

void updateReferences(struct PI * pi, float chi1_d){
    pi->chi1_d = chi1_d;
}

void calculateManipulation(struct PI * pi, float chi1){
  float error_d;
  float error_i;
  float u;

  pi->prev_error   = pi->error;
  pi->error        = pi->chi1_d- chi1;

  error_d = (pi->error- pi->prev_error) / pi->sample_time;
  error_i = ((pi->error+ pi->prev_error) / 2 * pi->sample_time) + pi->error;

  u  = pi->k_p* pi->error+ pi->k_i* error_i + pi->k_d* error_d;
  // printf("U: %3.3f, K{%3.3f, %3.3f, %3.3f}\r\n",u, pi->error, error_i, error_d);
                                                              
  if(!isnan(u) || u != 0.0)
      pi->u= u;
}

void saturateManipulation(struct PI * pi, float chi1){
  calculateManipulation(pi, chi1);
  pi->u = abs(pi->u) > pi->U_MAX ? pi->u / abs(pi->u) * pi->U_MAX : pi->u;
  pi->u = pi->u < pi->U_MIN ? pi->U_MIN : pi->u;
}

#include <stdio.h>