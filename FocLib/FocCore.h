#ifndef __FOCCORE_H__
#define __FOCCORE_H__

#include "stm32g4xx_hal.h"
#include <math.h>
#include "FocVariable.h"
#include "FocInterface.h"
#include "PID.h"
#include "Filter.h"
#include "SMO.h"

/*应用宏定义*/
#define Param_Identify 1
#define FOCTS 0.00005f  //Foc运行周期 1/freq

/*常用数据定义*/
#define TS 0.00005 //中断回调周期
#define _2PI 6.2831853f
#define _3PI_2 4.7123889803f
#define SQRT_3 1.73205080757f

typedef enum
{
    MOTORINIT,// 电机初始化
    MOTORIDENTIFY,// 电机参数辨识
    MOTORREADY,// 电机就绪
    MOTORERROR,// 电机异常
}Motor_State;

typedef enum
{
    POSITIONCTR,// 位置控制
    SPEEDCTR,// 速度控制
    TORQUECTR,// 转矩控制
    OPENCTR,// 开环控制
}Foc_State;

typedef struct FocCore
{
    Foc_State state;
    uint8_t pos_loop_divider;
    uint8_t speed_loop_divider;
    float AngleEle;
    float sintheta;
    float costheta;
    float Ialpha;
    float Ibeta;
    float Ualpha;
    float Ubeta;
    float Iq;
    float Id;
    float Ealpha;
    float Ebeta;

    /*结构体定义*/
    PID_Struct TorquePID;
		PID_Struct IdPID;
    PID_Struct SpeedPID;
    PID_Struct PositionPID;
    LowPassFilter_t Iq_LPFilter;
    LowPassFilter_t Id_LPFilter;

}Foc_Def;


typedef struct 
{
    Motor_State state;
    uint8_t pole;           //*
    uint8_t Encoder_dir;
    uint16_t TimeCount;     //*
    uint16_t Iabc_raw[3];
    uint16_t Iabc_offset[3];

    float SupplyVolt;       //*
    float MaxSpeed;
    float MaxCurrent;
    float Rs;
    float Lq;
    float Ld;
    float flux;
    float AngleMech;
    float AngleMech_Offset;
    float AngleMech_acu;
    float Speed;
    float we;
    float Iabc[3];
    /*结构体*/
    Foc_Def MFoc;
    LowPassFilter_t Iabc_LPFilter[3];
    LowPassFilter_t Speed_LPFilter;
    MovingAverageFilter_t Speed_MovingFilter;
    SlidingModeObserver SMO;

}Motor_Def;




static float foc_cos(float angle);
static float foc_sin(float angle);
float _Foc_Current_Standardization(float current);
float _Foc_GetAngle_ele(float angle_mech, uint16_t pole);
void _Foc_inverseParkTransform(float Uq, float Ud, float *U_alpha, float *U_beta, float sintheta, float costheta);
void _Foc_clarkParkTransform(float Iabc[3], float angle_ele,float *I_alpha, float *I_beta, float *Iq, float *Id, float sintheta, float costheta);
void _Foc_svpwm(float Ualpha, float Ubeta, float Udc, float ARR,float *T1, float *T2, float *T);
void _Foc_SetThreePhaseVoltage(float Uq, float Ud, float Udc, float Arr, float *Ualpha, float *Ubeta,float sintheta, float costheta);
uint8_t _Foc_EncoderDirIdentify(Motor_Def *motor, float Ts, float Uq);
void _Foc_ParamIdentify(float Ts, float Angle_Ele, float Iabc[3], float Iq, float Id, float sintheta, float costheta);

void Motor_CTRMode(Motor_Def *motor, float Target);
void Motor_Run(Motor_Def *motor, uint8_t Command, float Target);

#endif
