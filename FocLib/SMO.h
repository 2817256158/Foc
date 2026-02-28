/**
 * @file sm_observer.h
 * @brief 滑膜观测器(Sliding Mode Observer)头文件
 * @details 用于表贴式永磁同步电机(PMSM)的无传感器控制
 * 数学原理:
 * - 电流观测器: di_hat/dt = (1/Ls)(u - Rs*i_hat - e_hat)
 * - 滑膜面: s = i - i_hat
 * - 反电势观测: e_hat = K*sat(s/phi) - alpha*(i - i_hat)
 * - 角度估算: theta_hat = -atan2(e_alpha_hat, e_beta_hat)
 * - 速度估算: omega_hat = d(theta_hat)/dt
 */

#ifndef _SMO_H
#define _SMO_H

#include <math.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "filter.h"


#ifdef __cplusplus
extern "C" {
#endif

// 数学常量定义 (如果平台没有定义M_PI)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692f
#endif

/**
 * @brief 滑膜观测器结构体
 */
typedef struct {
    // 电机参数
    float Rs;           ///< 定子电阻 (Ω)
    float Ld;           ///< 定子电感 (H) - 表贴式电机 Ld = Lq = Ls
    float Lq;
    float flux;     ///< 永磁体磁链 (Wb)
    float pole;        ///< 极对数
    float fs;           ///< 采样频率 (Hz)
    float Ts;           ///< 采样时间 (s) = 1/fs
    
    // 滑膜观测器参数
    float h;            ///< 滑膜增益
    
    // 状态变量
    float theta_hat;    ///< 估计的转子角度 (rad)
    float omega_hat;    ///< 估计的转子速度 (rad/s)
    float i_alpha_hat;  ///< 估计的α轴电流 (A)
    float i_beta_hat;   ///< 估计的β轴电流 (A)
    float e_alpha_hat;  ///< 估计的α轴反电势 (V)
    float e_beta_hat;   ///< 估计的β轴反电势 (V)
    
    // 内部变量
    float prev_theta_hat; ///< 上一时刻的角度估计


    LowPassFilter_t e_alpha_LPF;
    LowPassFilter_t e_beta_LPF;
    
} SlidingModeObserver;

// 全局PLL参数结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float WeForeLPFFactor; // 角速度低通滤波系数
    float Ts;           // 采样时间
    int Dir;            // 旋转方向 (1: 正转, -1: 反转)
} PLL_PARAMS;
// PLL状态结构体（需要保持的状态变量）
typedef struct {
    float IPart;             // 积分项累计值
    float WeForeLPF;         // 低通滤波后的角速度
    float ThetaFore;         // 预估角度
    float ThetaCompensate;   // 补偿角度
    float SinVal;
    float CosVal;
} PLL_STATE;

typedef struct
{  	
	float Ts;	                  //µ÷ÓÃÖÜÆÚ	
	float Rs;                   //Ïàµç×è
	float Ld;                   //Ïàµç¸Ð
  float Gain;                 //»¬Ä¤¹Û²âÆ÷ÔöÒæ
	
  float Ialpha;               //¦ÁÖáÊµ¼ÊµçÁ÷
	float Ibeta;                //¦ÂÖáÊµ¼ÊµçÁ÷		
	
  float IalphaFore;           //¦ÁÖáÔ¤²âµçÁ÷
	float IbetaFore;            //¦ÂÖáÔ¤²âµçÁ÷
	
  float Ualpha;               //¦ÁÖáÊµ¼ÊµçÑ¹
	float Ubeta;                //¦ÂÖáÊµ¼ÊµçÑ¹	
	
	float EalphaFore;           //¦ÁÖáÔ¤²â·´ÊÆ
	float EalphaForeLPF;        //¦ÁÖáÔ¤²â·´ÊÆÂË²¨Öµ	
	float EalphaForeLPFFactor;  //¦ÁÖáÔ¤²â·´ÊÆÂË²¨ÏµÊý
 
	float EbetaFore;            //¦ÂÖáÔ¤²â·´ÊÆ
	float EbetaForeLPF;         //¦ÂÖáÔ¤²â·´ÊÆÂË²¨Öµ
	float EbetaForeLPFFactor;   //¦ÂÖáÔ¤²â·´ÊÆÂË²¨ÏµÊý
}SMO_STRUCT;

void SMO_Init(SlidingModeObserver* observer, float Rs, float Ld, float Lq, float pole, float flux, float h, float Ts);
void SMO_Updata(SlidingModeObserver* observer, float Ualpha, float Ubeta, float we, 
                float Ialpha, float Ibeta, float Iq, float Id);
            
void SMO_GainIdentify(SlidingModeObserver *observer, float Ealpha_hat, float Ebeta_hat, float Ealpha, float Ebeta, float *h);
float get_electrical_angle(float Ealpha, float Ebeta, float offset_theta);
void SMO_Calculate(SMO_STRUCT *p);
#endif // SM_OBSERVER_H