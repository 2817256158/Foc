#include "FocInterface.h"

#include "TLE5012B.h"
#include "Filter.h"
#include "FocVariable.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;

MovingAverageFilter_t SpeedFilter;
float SpeedFilterBuff[20];

void __FocInterface_Init(void)
{
    MovingAverageFilter_Init(&SpeedFilter, SpeedFilterBuff, 20, 20);
}

void __Foc_Delayms(uint16_t ms)
{
    HAL_Delay(ms);
}
void __Foc_SetPwm(float Tcmp1, float Tcmp2, float Tcmp3)
{
    TIM1->CCR1 = (uint16_t)Tcmp1;
    TIM1->CCR2 = (uint16_t)Tcmp2;
    TIM1->CCR3 = (uint16_t)Tcmp3;
}
void __Foc_GetCurrent(uint16_t *Ia, uint16_t *Ib, uint16_t *Ic)
{
    /*获取原始电流数值*/
    *Ia = hadc1.Instance -> JDR1;
    *Ib = hadc1.Instance -> JDR2;
    *Ic = hadc1.Instance -> JDR3;
}
/*获取角度*/
void __Foc_GetAngle(float *angle)
{
    /*获取角度*/
    *angle = ReadAngle_Rad();
}


// 全局变量用于存储状态
static float previous_angle = 0.0f;
static float accumulated_angle = 0.0f;
static int first_update = 1;

/**
 * @brief 角度累积计算函数
 * @param Angle_Acu 输出参数，返回累积角度值
 * @param Angle_Mech 输入参数，当前机械角度值（0-2π范围内）
 */
void __Foc_GetAngle_Acu(float *Angle_Acu, float Angle_Mech)
{
    if (first_update) {
        // 第一次调用，初始化
        previous_angle = Angle_Mech;
        accumulated_angle = 0.0f;
        first_update = 0;
        *Angle_Acu = accumulated_angle;
        return;
    }
    
    // 计算角度差
    float angle_diff = Angle_Mech - previous_angle;
    
    // 处理角度突变情况（0↔2π边界）
    if (angle_diff < -M_PI) {
        // 正向跨越2π→0边界
        angle_diff += 2 * M_PI;
    } else if (angle_diff > M_PI) {
        // 反向跨越0→2π边界
        angle_diff -= 2 * M_PI;
    }
    
    // 更新累积角度
    accumulated_angle += angle_diff;
    
    // 保存当前角度用于下一次计算
    previous_angle = Angle_Mech;
    
    // 输出累积角度
    *Angle_Acu = accumulated_angle;
}

/*速度获取接口*/
/*获取速度
 * @param Speed: 输出参数，角速度值，单位：弧度/秒 (rad/s)
 * @param Angle: 输入参数，当前角度值，单位：弧度 (rad)，范围：0-2π
 * @param Ts: 输入参数，时间间隔，单位：秒 (s)
 */
void __Foc_GetSpeed(float *Speed, float Angle, float Ts)
{
    static float Angle_prv = 0.0f;
    static uint8_t first_call = 1;
    
    // 第一次调用时，初始化前一个角度值
    if (first_call) {
        Angle_prv = Angle;
        first_call = 0;
        *Speed = 0.0f;
        return;
    }
    
    // 计算角度差，处理周期性跳变
    float angle_diff = Angle - Angle_prv;
    
    // 处理角度突变：如果角度差大于π，说明发生了周期跳变
    if (angle_diff > 3.14159f) {  // π ≈ 3.14159
        angle_diff -= 6.28318f;   // 2π ≈ 6.28318
    } else if (angle_diff < -3.14159f) {
        angle_diff += 6.28318f;
    }
    
    // 计算角速度：角度差 / 时间间隔，单位：弧度/秒 (rad/s)
    *Speed = angle_diff / Ts;
    g_Speed = *Speed;

    /*低通滤波*/
    *Speed = MovingAverageFilter_Process(&SpeedFilter, *Speed);
    // 更新前一个角度值
    Angle_prv = Angle;
}
void __Foc_GetVbus(float *Vbus)
{

}
void __Foc_GetTemperature(float *Temperature)
{

}
void __Foc_GetCommand(uint8_t Command, float Param)/*通过16进制编码实现命令控制 @param1:命令类型 @param2:命令参数*/
{

}
