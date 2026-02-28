#ifndef _NLFO_H_
#define _NLFO_H_

#include "stm32g4xx_hal.h"
#include "FocCore.h"

typedef struct
{
    float Kp;
    float Ki;
    float err;
    float err_acu;
    float err_prv;
}NLFO_PLL;


typedef struct 
{
    float Gain;
    float x[2];
    float y[2];
    float y_dot[2];
    float flux_alpha;
    float flux_beta;
    float omega_hat;
    float theta_hat;
    NLFO_PLL pll;

}NLFO_Observer;


void NLFO_Observer_Updata(NLFO_Observer *observer, Motor_Def *motor);
void NLFO_PLL_Updata(NLFO_PLL *pll, NLFO_Observer *observer);

#endif