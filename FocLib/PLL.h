#ifndef PLL_H
#define PLL_H

#include <math.h> // 用于cos、sin函数

// PLL可配置参数（采样周期、PID增益等）
typedef struct {
    float Ts;       // 采样周期（s）
    float Kp;       // PID比例增益（无量纲）
    float Ki;       // PID积分增益（1/s）
    float Kd;       // PID微分增益（s）
    float K;        // 积分模块增益（无量纲）
} PLL_Params;

// PLL状态（需存储的历史变量）
typedef struct {
    PLL_Params params;   // PLL参数
    float e_prev;        // 上一时刻误差（e(k-1)）
    float e_sum;         // 误差累积和（sum(e(0)到e(k))）
    float theta_hat;     // 当前相位角估计（rad）
    float we_hat;        // 当前角频率估计（rad/s，等于cos(theta_hat)）
} PLL_Struct;

// 初始化PLL（设置参数和初始状态）
void PLL_Init(PLL_Struct *handle, const PLL_Params *params);

// 更新PLL（输入α/β误差，输出相位/角频率估计）
// 输入：handle - PLL句柄；Ealpha_hat - α相误差；Ebeta_hat - β相误差
// 输出：theta_hat - 相位角估计；we_hat - 角频率估计
void PLL_Update(PLL_Struct *handle, float Ealpha_hat, float Ebeta_hat, 
                float *theta_hat, float *we_hat);

#endif // PLL_H