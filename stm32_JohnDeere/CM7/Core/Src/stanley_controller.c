/** ----------------------------------------------------------------------------
 * @file: stanley_controller.c
 * @date: November 24th, 2023
 * @author: Max Pacheco
 * @email: maxprpxam@icloud.com
 *
 * @brief: Stanley Controller class
 * Obtained from vanttec_controllers repository
 * -----------------------------------------------------------------------------
 * */

#include "stanley_controller.h"

void initStanley(struct Stanley * stanley, float *delta_sat, float k, float k_soft)
{
    stanley->sat[0] = delta_sat[0];
    stanley->sat[1] = delta_sat[1];
    stanley->psi = 0;
    stanley->k = k;
    stanley->k_soft = k_soft;
}

void calculateCrosstrackError(struct Stanley * stanley, struct Point * vehicle_pos, struct Point * p1, struct Point * p2){
    float m1;
    float m2;
    float b;
    float c;
    float xp;
    float yp;

    float ex = p2->x - p1->x;
    float ey = p2->y - p1->y;

    // Angle of path frame
    stanley->ak = atan2(ey,ex);

    if(isnormal(ex) && isnormal(ey)){
        // Slope of path
        m1 = ex/ey;
        b = p2->x - m1*p2->y;

        // Slope of normal line to the path
        m2 = -1/m1;
        c = vehicle_pos->x - m2*vehicle_pos->y;

        // Obtain intersection point
        yp = (c - b)/(m1 - m2);
        xp = m1*yp + b;

    } else {
        if(!isnormal(ex)){
            yp = vehicle_pos->y;
            xp = p2->x; // or x1
        }
        if(!isnormal(ey)){
            yp = p2->y; // or y1
            xp = vehicle_pos->x;
        }
    }

    // Along-track and crosstrack errors in path frame
    stanley->e_a = (p2->x - xp) * cos(stanley->ak) + (p2->y - yp) * sin(stanley->ak); // along-track error
    stanley->e_c = -(vehicle_pos->x - xp) * sin(stanley->ak) + (vehicle_pos->y - yp) * cos(stanley->ak); // crosstrack error
}

void setYawAngle(struct Stanley * stanley, float psi){
    stanley->psi = psi;
}

void calculateSteering(struct Stanley * stanley, float vel, uint8_t precision){
    stanley->vel = vel;

    // PI error fixed due to rounding in ak_ angle when the path is vertical that makes it greater than M_PI
    double PI = M_PI + 1e-3;
    if(stanley->ak >= PI/2 && stanley->ak <=  PI && stanley->psi <= -PI/2 && stanley->psi >= - PI){
        stanley->psi = stanley->psi + PI*2;
    } else if (stanley->ak < -PI/2 && stanley->ak > - PI && stanley->psi > PI/2 && stanley->psi <  PI){
        stanley->psi = stanley->psi - PI*2;
    }

    float phi = stanley->psi - stanley->ak;
    stanley->delta = phi + atan2(stanley->k*stanley->e_c,stanley->k_soft + stanley->vel);

    // You want to reduce psi by delta so ...
    stanley->delta = -stanley->delta;

    stanley->delta = stanley->delta < stanley->sat[1] ? stanley->sat[1] : stanley->delta;
    stanley->delta = stanley->delta > stanley->sat[0] ? stanley->sat[0] : stanley->delta;
}