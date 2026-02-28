#include "SMO.h"

#ifndef _2PI
#define _2PI 6.2831852f
#endif


float sign(float num)
{
    if(num > 0)return 1;
    else return -1;
}

// 饱和函数代替符号函数
float saturate(float value, float boundary)
{
    if (value > boundary) return 1.0f;
    else if (value < -boundary) return -1.0f;
    else return value / boundary;
}
void SMO_Init(SlidingModeObserver* observer, float Rs, float Ld, float Lq, float pole, float flux, float h, float Ts)
{
    observer->h = h;
    observer->Rs = Rs;
    observer->Ld  = Ld;
    observer->Lq = Lq;
    observer->pole = pole;
    observer->flux = flux;
    observer->Ts = Ts;

    observer->e_alpha_hat = 0.0f;
    observer->e_beta_hat = 0.0f;
    observer->i_alpha_hat = 0.0f;
    observer->i_beta_hat = 0.0f;

    LowPassFilter_Init(&observer->e_alpha_LPF, 0.01f);
    LowPassFilter_Init(&observer->e_beta_LPF, 0.01f);
}
float factor1 = 0.0f;
float factor2 = 0.0f;
float factor3 = 0.0f;
float factor4 = 0.0f;
float temp1 = 0.0f;
float temp2 = 0.0f;
float buff[50] = {0};
uint8_t index = 0;
float boundary = 0.5f;
void SMO_Updata(SlidingModeObserver* observer, float Ualpha, float Ubeta, float we, 
                float Ialpha, float Ibeta, float Iq, float Id)
{
    //float factor = observer->Rs / observer->Ld = 4250.0f;
    temp1 = (Ualpha / observer->Ld) - 4250.0f * observer->i_alpha_hat - observer->e_alpha_hat / observer->Ld; 
    temp2 = (Ubeta / observer->Ld) - 4250.0f * observer->i_beta_hat - observer->e_beta_hat / observer->Ld;
    /*积分得到Ibeta_hat Ialpha_hat*/
    
    observer->i_alpha_hat += (temp1*observer->Ts);
    observer->i_beta_hat += (temp2*observer->Ts);
    /*计算得到Ealpha_hat Ebeta_hat*/
    observer->e_alpha_hat = (saturate(observer->i_alpha_hat - Ialpha, boundary) * observer->h);
    observer->e_beta_hat = (saturate(observer->i_beta_hat - Ibeta, boundary) * observer->h);
    /*进行低通滤波*/
    observer->e_alpha_hat = LowPassFilter_Process(&observer->e_alpha_LPF, observer->e_alpha_hat);
    observer->e_beta_hat = LowPassFilter_Process(&observer->e_beta_LPF, observer->e_beta_hat);


}

void SMO_GainIdentify(SlidingModeObserver *observer, float Ealpha_hat, float Ebeta_hat, float Ealpha, float Ebeta, float *h)
{
    float sign_Ealpha_hat = sign(Ealpha_hat);
    float sign_Ebeta_hat = sign(Ebeta_hat);
    float temp[8] = {0};
    temp[0] = fabs(Ealpha_hat)*-observer->Rs;
    //temp[1] = sign_Ealpha_hat * Ebeta_hat;
    temp[2] = Ealpha * sign_Ealpha_hat;
    temp[3] = temp[0] + temp[2];

    temp[4] = fabs(Ebeta_hat)*-observer->Rs;
    //temp[5] = sign_Ebeta_hat * Ealpha_hat;
    temp[6] = Ebeta * sign_Ebeta_hat;
    temp[7] = temp[4] + temp[6];

    h[0] = temp[3];
    h[1] = temp[7];

    //observer->h = fmax(fmax(fabs(h[0]), observer->h), fabs(h[1])); 
}
float get_electrical_angle(float Ealpha, float Ebeta, float offset_theta) {
    // 1. 计算原始角度（范围：-π ~ π 弧度）
    float raw_angle = atan2(Ebeta, Ealpha);
    
    // 2. 将角度转换到 0 ~ 2π 范围
    float angle_0_2pi = (raw_angle < 0) ? (raw_angle + 2 * M_PI) : raw_angle;
    
    // 3. 叠加相位补偿角
    float compensated_angle = angle_0_2pi + offset_theta;
    
    // 4. 确保补偿后的角度仍在 0 ~ 2π 范围
    if (compensated_angle >= _2PI) {
        compensated_angle -= _2PI;
    } else if (compensated_angle < 0) {
        compensated_angle += _2PI;
    }
    
    return compensated_angle;
}
void SMO_Calculate(SMO_STRUCT *p)
{
//*************************************²ÎÊýÔ¤²â**********************************/  	
	p->IalphaFore += p->Ts * (-p->Rs / p->Ld * p->IalphaFore + (p->Ualpha - p->EalphaForeLPF) / p->Ld);
	p->IbetaFore  += p->Ts * (-p->Rs / p->Ld * p->IbetaFore  + (p->Ubeta  - p->EbetaForeLPF)  / p->Ld);	
	
	if     ((p->IalphaFore - p->Ialpha) > 1.0f) p->EalphaFore = p->Gain;
	else if((p->IalphaFore - p->Ialpha) < -1.0f) p->EalphaFore = -p->Gain;	
	else     p->EalphaFore = p->Gain * (p->IalphaFore - p->Ialpha);
	
	if     ((p->IbetaFore - p->Ialpha) > 1.0f) p->EbetaFore = p->Gain;
	else if((p->IbetaFore - p->Ialpha) < -1.0f) p->EbetaFore = -p->Gain;	
	else     p->EbetaFore = p->Gain * (p->IbetaFore - p->Ibeta);
	
	p->EalphaForeLPF = p->EalphaFore * p->EalphaForeLPFFactor + p->EalphaForeLPF * (1 - p->EalphaForeLPFFactor); 
	p->EbetaForeLPF  = p->EbetaFore  * p->EbetaForeLPFFactor  + p->EbetaForeLPF  * (1 - p->EbetaForeLPFFactor);	
}