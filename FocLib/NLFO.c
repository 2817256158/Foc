#include "NLFO.h"

float _pow(float num)
{
    return num*num;
}

float NLFO_err = 0.0f;
float bufft[100] = {0};
uint8_t indext = 0;
void NLFO_Observer_Updata(NLFO_Observer *observer, Motor_Def *motor)
{
    NLFO_err = _pow(motor->flux) - _pow(observer->flux_alpha) - _pow(observer->flux_beta);
    observer->y_dot[0] = motor->MFoc.Ualpha - motor->Rs * motor->MFoc.Ialpha + NLFO_err * observer->flux_alpha * observer->Gain;
    observer->y_dot[1] = motor->MFoc.Ubeta - motor->Rs * motor->MFoc.Ibeta + NLFO_err * observer->flux_beta * observer->Gain;
    observer->y[0] += observer->y_dot[0];
    observer->y[1] += observer->y_dot[1];
    if(indext < 99)
    bufft[indext++] = NLFO_err;

    observer->flux_alpha = -motor->MFoc.Ialpha * motor->Ld + observer->y[0];
    observer->flux_beta = -motor->MFoc.Ibeta * motor->Lq + observer->y[1];

}

void NLFO_PLL_Updata(NLFO_PLL *pll, NLFO_Observer *observer)
{
    observer->pll.err = observer->flux_beta * cosf(observer->theta_hat) - observer->flux_alpha * sinf(observer->theta_hat);
    observer->pll.err_acu += observer->pll.err;

    observer->omega_hat = observer->pll.Kp * observer->pll.err + observer->pll.Ki * (observer->pll.err_acu);
    observer->theta_hat += observer->omega_hat;
}