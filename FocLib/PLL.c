#include "PLL.h"
#include <stddef.h> // 用于NULL检查
#include <math.h>

// 若编译器未定义M_PI，手动定义（如Keil、MSVC等）
#ifndef _2PI
#define _2PI 6.2831852f
#endif


/***
 * 电机最高转速4000rpm = 67 rps  极对数是7
 * 电角度变化率为 67*7 = 470Hz => wn>470 => 取480作为带宽
 * 
 * k=[(Ld - Lq)(weLd - d(iq)/dt) + we*flux] 由Ld = Lq => k = we*flux
 * Ki = wn²/(k*flux)
 * 设计在500rpm时切观测器 =>k = 4.0299 => ki = 480² / (500 * 0.0057) = 80842
 * Kp = 2*0.707*sqrt(k*Ki)/k
 * 阻尼比取 0.707 欠阻尼 => Kp = 201
*/

// 初始化PLL（初始状态设为合理值）
void PLL_Init(PLL_Struct *handle, const PLL_Params *params) {
    if (handle == NULL || params == NULL) return;
    
    // 复制参数
    handle->params = *params;
    
    // 初始化状态（0或合理初始值）
    handle->e_prev = 0.0f;       // 上一时刻误差
    handle->e_sum = 0.0f;        // 误差累积和
    handle->theta_hat = 0.0f;    // 初始相位角（rad）
    handle->we_hat = 1.0f;       // 初始角频率（cos(0)=1）
}

// void PLL_Update(PLL_Struct *handle, float Ealpha_hat, float Ebeta_hat, 
//                 float *theta_hat, float *we_hat) {
//     if (handle == NULL || theta_hat == NULL || we_hat == NULL) return;
    
//     // 获取当前状态和参数
//     float e_prev = handle->e_prev;           // e(k-1)
//     float e_sum = handle->e_sum;             // sum(e(0)到e(k-1))
//     float theta_prev = handle->theta_hat;    // θ̂(k-1)
//     float we_prev = handle->we_hat;          // ω̂_e(k-1)
//     const PLL_Params *params = &handle->params;
//     float Ts = params->Ts;
//     float Kp = params->Kp;
//     float Ki = params->Ki;
//     float Kd = params->Kd;
//     float K = params->K;
    
//     // 1. 计算当前误差e(k)（αβ→dq变换的d轴分量）
//     float cos_theta = cosf(theta_prev);      // cos(θ̂(k-1))
//     float sin_theta = sinf(theta_prev);      // sin(θ̂(k-1))
//     float e = Ealpha_hat * cos_theta - Ebeta_hat * sin_theta;
    
//     // 2. 计算离散PID输出u(k)（位置式）
//     float u = Kp * e + Ki * (e_sum + e) + Kd * (e - e_prev);
    
//     // 3. 求和节点：s(k) = u(k) + ω̂_e(k-1)
//     float s = u + we_prev;
    
//     // 4. 积分模块：更新相位角θ̂(k)
//     float theta_current = theta_prev + K * Ts * s;
    
//     // ---------- 新增：相位角周期限制（[0, 2π)） ----------
//     theta_current = fmodf(theta_current, _2PI); // 取模得到[-2π, 2π)
//     if (theta_current < 0.0f) {                       // 修正为[0, 2π)
//         theta_current += _2PI;
//     }
//     // ------------------------------------------------------
    
//     // 5. 更新角频率ω̂_e(k) = cos(θ̂(k))（根据框图）
//     float we_current = cosf(theta_current);
    
//     // 6. 保存状态（用于下一时刻）
//     handle->e_prev = e;              // 更新上一时刻误差
//     handle->e_sum = e_sum + e;       // 更新误差累积和
//     handle->theta_hat = theta_current; // 更新相位角（已限制周期）
//     handle->we_hat = we_current;     // 更新角频率
    
//     // 7. 输出结果
//     *theta_hat = theta_current;
//     *we_hat = we_current;
// }
float offset = 3.14159f;
void PLL_Update(PLL_Struct *handle, float Ealpha_hat, float Ebeta_hat, 
                float *theta_hat, float *we_hat) {
    if (handle == NULL || theta_hat == NULL || we_hat == NULL) return;
    
    // 获取当前状态和参数
    float e_prev = handle->e_prev;           // e(k-1)
    float e_sum = handle->e_sum;             // sum(e(0)到e(k-1))
    float theta_prev = handle->theta_hat;    // θ̂(k-1)
    float we_prev = handle->we_hat;          // ω̂_e(k-1)
    const PLL_Params *params = &handle->params;
    float Ts = params->Ts;
    float Kp = params->Kp;
    float Ki = params->Ki;
    float Kd = params->Kd;
    float K = params->K;
    
    // 1. 计算当前误差e(k)（修正Ebeta项的符号：从减号改为加号）
    float cos_theta = cosf(theta_prev);      // cos(θ̂(k-1))
    float sin_theta = sinf(theta_prev);      // sin(θ̂(k-1))
    float e = -Ealpha_hat * cos_theta - Ebeta_hat * sin_theta;  
    
    // 2. 计算离散PID输出u(k)（位置式）
    float u = Kp * e + Ki * (e_sum + e) + Kd * (e - e_prev);
    
    // 3. 求和节点：s(k) = u(k) + ω̂_e(k-1)
    float s = u + we_prev;
    
    // 4. 积分模块：更新相位角θ̂(k)
    float theta_current = theta_prev + K * Ts * s;
    
    // ---------- 相位角周期限制（[0, 2π)） ----------
    theta_current = fmodf(theta_current, _2PI); // 取模得到[-2π, 2π)
    if (theta_current < 0.0f) {                       // 修正为[0, 2π)
        theta_current += _2PI;
    }

    // ------------------------------------------------------
    
    // 5. 更新角频率ω̂_e(k) = cos(θ̂(k))（根据框图）
    float we_current = cosf(theta_current);
    
    // 6. 保存状态（用于下一时刻）
    handle->e_prev = e;              // 更新上一时刻误差
    handle->e_sum = e_sum + e;       // 更新误差累积和
    handle->theta_hat = theta_current; // 更新相位角（已限制周期）
    handle->we_hat = we_current;     // 更新角频率
    
    // 7. 输出结果
    *theta_hat = theta_current;
    *we_hat = we_current;
}