#include "FocCore.h"

#ifdef USE_ARM_MATH
#include "arm_math.h"
static float foc_cos(float angle)
{
    return arm_cos_f32(angle);
}
static float foc_sin(float angle)
{
    return arm_sin_f32(angle);
}
#else
static float foc_cos(float angle)
{
    return cos(angle);
}
static float foc_sin(float angle)
{
    return sin(angle);
}
#endif

#define _PeriodLimit(MaxNum, Num) (Num > MaxNum ? MaxNum : Num)

/*电流采样增益参数*/
#define Gain 20.0f
#define Shunt 0.002f
#define ADC_Factor 0.000805f // 3.3/4096
#define Gain_Factor (Gain * Shunt)

float _Foc_Current_Standardization(float current)
{
    return (current * ADC_Factor / Gain_Factor);
}

/**
 ******************************************************************************
 * @file    FocCore.c
 * @author  Mr.Zhang
 * @brief   文件提供FOC所需最基本的运算
 ******************************************************************************
 * @attention
 *
 *
 *
 *
 *
 *
 *
 *
 *
 ******************************************************************************
 */

float _Foc_GetAngle_ele(float angle_mech, uint16_t pole)
{
    float angle_ele = 0.0f;
    angle_ele = angle_mech * pole;
    angle_ele = fmod(angle_ele, _2PI);
    return angle_ele;
}

void _Foc_inverseParkTransform(float Uq, float Ud, float *U_alpha, float *U_beta, float sintheta, float costheta)
{
    *U_alpha = Ud * costheta - Uq * sintheta;
    *U_beta = Ud * sintheta + Uq * costheta;
}

void _Foc_clarkParkTransform(float Iabc[3], float angle_ele,
                             float *I_alpha, float *I_beta, float *Iq, float *Id, float sintheta, float costheta)
{
    *I_alpha = Iabc[0];
    *I_beta = (Iabc[0] + 2 * Iabc[1]) * 0.5773502f;
    *Id = *I_alpha * costheta + *I_beta * sintheta;
    *Iq = -*I_alpha * sintheta + *I_beta * costheta;
}

void _Foc_svpwm(float Ualpha, float Ubeta, float Udc, float ARR,
                float *T1, float *T2, float *T3)
{
    float t1 = 0.0f, t2 = 0.0f;
    float ta = 0.0f, tb = 0.0f, tc = 0.0f;
    float Vref1, Vref2, Vref3;
    float X, Y, Z;
    int sector = 0;

    // 电压归一化处理
    float norm_alpha = Ualpha / Udc;
    float norm_beta = Ubeta / Udc;

    Vref1 = norm_beta;
    Vref2 = (SQRT_3 * norm_alpha - norm_beta);
    Vref3 = (-SQRT_3 * norm_alpha - norm_beta);

    if (Vref1 > 0)
        sector = 1;
    if (Vref2 > 0)
        sector += 2;
    if (Vref3 > 0)
        sector += 4;

    // 使用ARR代替Ts（归一化计算）
    X = SQRT_3 * norm_beta;
    Y = 1.5f * norm_alpha + 0.8660254f * norm_beta;
    Z = -1.5f * norm_alpha + 0.8660254f * norm_beta;

    switch (sector)
    {
    case 1:
        t1 = Z;
        t2 = Y;
        break;
    case 2:
        t1 = Y;
        t2 = -X;
        break;
    case 3:
        t1 = -Z;
        t2 = X;
        break;
    case 4:
        t1 = -X;
        t2 = Z;
        break;
    case 5:
        t1 = X;
        t2 = -Y;
        break;
    case 6:
        t1 = -Y;
        t2 = -Z;
        break;
    }

    // 饱和处理（直接使用归一化值）
    float sum_t = t1 + t2;
    if (sum_t > 1.0f)
    {
        t1 = ARR * t1 / sum_t;
        t2 = ARR * t2 / sum_t;
    }
    else
    {
        t1 *= ARR;
        t2 *= ARR;
    }

    // 计算时间分配（基于ARR比例）
    float t0 = (ARR - (t1 + t2)) / 4.0f;
    tb = t0 + t1 / 2.0f;
    tc = tb + t2 / 2.0f;

    // 根据扇区分配比较值
    switch (sector)
    {
    case 1:
        *T1 = tb;
        *T2 = t0;
        *T3 = tc;
        break;
    case 2:
        *T1 = t0;
        *T2 = tc;
        *T3 = tb;
        break;
    case 3:
        *T1 = t0;
        *T2 = tb;
        *T3 = tc;
        break;
    case 4:
        *T1 = tc;
        *T2 = tb;
        *T3 = t0;
        break;
    case 5:
        *T1 = tc;
        *T2 = t0;
        *T3 = tb;
        break;
    case 6:
        *T1 = tb;
        *T2 = tc;
        *T3 = t0;
        break;
    }

    // 直接饱和处理
    *T1 = (*T1 > ARR) ? ARR : *T1;
    *T2 = (*T2 > ARR) ? ARR : *T2;
    *T3 = (*T3 > ARR) ? ARR : *T3;
}

void _Foc_SetThreePhaseVoltage(float Uq, float Ud, float Udc, float Arr, float *Ualpha, float *Ubeta, float sintheta, float costheta)
{
    float T[3] = {0};

    _Foc_inverseParkTransform(Uq, Ud, Ualpha, Ubeta, sintheta, costheta);
    _Foc_svpwm(*Ualpha, *Ubeta, Udc, Arr, &T[0], &T[1], &T[2]);

    T[0] = _PeriodLimit(Arr - 5, T[0]);
    T[1] = _PeriodLimit(Arr - 5, T[1]);
    T[2] = _PeriodLimit(Arr - 5, T[2]);

    g_T[0] = T[0];
    g_T[1] = T[1];
    g_T[2] = T[2];

    __Foc_SetPwm(T[0], T[1], T[2]);
}
uint16_t Tick = 0; // 时间记录
float Ud = 0.0f;
float Ud_recode[2] = {0};
float Current_recode[2] = {0};
float Rs = 0.0f;
float Ls = 0.0f;
float Ld = 0.0f;
float Lq = 0.0f;
uint8_t Resistance_State = 0;
uint8_t Inductor_State = 0;
uint16_t IdentifyNum = 0;
float yuzhi[2] = {3.0f, 5.0f};
volatile uint8_t IdentifyState = 0; // 状态标志位
float U[2] = {0};
float C_Id[110] = {0};
void _Foc_ParamIdentify(float Ts, float Angle_Ele, float Iabc[3], float Iq, float Id, float sintheta, float costheta)
{
    //    static uint16_t Tick = 0;// 时间记录
    //    static uint8_t IdentifyState = 0;// 状态标志位

    /*获取必要数据*/

    switch (IdentifyState)
    {
    case 0: // 识别前准备
        _Foc_SetThreePhaseVoltage(0.0f, 0.0f, 12.0f, 4200, &U[0], &U[1], sintheta, costheta);
        if (Tick <= 5000)
            Tick++;
        else // 电流稳定后进入电阻识别
        {
            Tick = 0;
            IdentifyState = 1;
        }
        break;
    case 1: // 电阻识别
        // static float Resistance_State = 0;
        if (Resistance_State == 0) // 到达目标电流1
        {
            if (-Iabc[0] >= yuzhi[0])
            {
                Resistance_State = 1;
                Ud_recode[0] = Ud; // 记录电压
            }
            else
                Ud += 0.00001;
        }
        else if (Resistance_State == 1)
        {
            if (Tick <= 5000)
                Tick++;
            else if (Tick <= 5100) // 记录电流值
            {
                Tick++;
                Current_recode[0] += Iabc[0];
            }
            else // 记录100组后计算平均电流 跳转第二步
            {
                Current_recode[0] /= 100.0f;
                Tick = 0;
                Resistance_State = 2;
            }
        }
        else if (Resistance_State == 2) // 达到目标电流2
        {
            if (-Iabc[0] >= yuzhi[1])
            {
                Resistance_State = 3;
                Ud_recode[1] = Ud; // 记录电压
            }
            else
                Ud += 0.00001;
        }
        else if (Resistance_State == 3)
        {
            if (Tick <= 5000)
                Tick++;
            else if (Tick <= 5100) // 记录电流值
            {
                Tick++;
                Current_recode[1] += Iabc[0];
            }
            else // 记录100组后计算平均电流 跳转第二步
            {
                Current_recode[1] /= 100.0f;
                Tick = 0;
                Resistance_State = 4;
            }
        }
        else if (Resistance_State == 4) // 电阻计算
        {
            Rs = (Ud_recode[1] - Ud_recode[0]) / (Current_recode[1] - Current_recode[0]);
            Ud = 0;
            Tick = 0;
            Resistance_State = 0;
            IdentifyState = 2; // 跳转到电感识别
        }

        break;
    case 2: // 电感识别

        if (Inductor_State == 0) // 准备阶段，等待电流降下来
        {
            if (Iabc[0] <= 0.5f && Iabc[0] >= -0.5f)
                Inductor_State = 1;
            Tick = 0;
        }
        else if (Inductor_State == 1)
        {
            /*方波注入法*/
            // static uint8_t flag_sign = 0;
            // static float last_Id = 0;
            // // 正负切换
            // if(flag_sign)Ud = 5, flag_sign = 0;
            // else Ud = -5, flag_sign = 1;

            // if(Tick < 100) last_Id = Iabc[0];// 过滤掉前面不稳定的数据
            // else if(Tick <= 200)
            // {
            //     C_Id[Tick-100] = Iabc[0];//记录电流
            //     //if(Tick % 20 == 0)
            //     //{
            //         float delt_Id = Iabc[0] - last_Id;
            //         Ls += (5.0f*0.00005f)/delt_Id;
            //         last_Id = Iabc[0];
            //     //}
            // }
            // else last_Id = 0.0f, Ud = 0.0f, Inductor_State = 2;
            // Tick ++;

            /*阶跃响应法*/
            Ud = Ud_recode[1];
            Tick++;
            if (-Iabc[0] >= -Current_recode[1] * 0.95f)
            {
                IdentifyNum++;
                // 计算电感值
                // Ls += (0.170*Tick*0.00005f)/(2.995f);
                // Ls += 0.170f * 2.0f *0.632f * 0.00005f * Tick;
                Ls += (0.170 * Tick * 0.00005f) / (2.3f);
                Inductor_State = 0;
                Ud = 0;
                Tick = 0;
                if (IdentifyNum == 100)
                    Inductor_State = 2;
            }
        }
        else if (Inductor_State == 2) // 计算电感
        {
            // 计算电感平均值
            Ls /= 100.0f;
            Ld = Ls * 1.5f;
            Lq = Ls * 1.5f;

            Tick = 0;
            IdentifyNum = 0;
            Inductor_State = 0;
            IdentifyState = 3; // 跳转到磁链辨识
        }

        break;
    case 3: // 磁链辨识

        break;
    default:
        break;
    }

    _Foc_SetThreePhaseVoltage(0.0f, Ud, 12.0f, 4200, &U[0], &U[1], sintheta, costheta);
}
float test = 0.0f;
void Motor_CTRMode(Motor_Def *motor, float Target)
{
    static uint16_t times = 0;
    float PositionTarget = 0.0f;
    float SpeedTarget = 0.0f;
    float TorqueTarget = 0.0f;
    float UqOutput = 0.0f;
    float UdOutput = 0.0f;
    switch (motor->MFoc.state)// ####注意:此处switch未使用break,故三环在switch中的位置不能换，属于嵌套关系####
    {
    case POSITIONCTR:
        PositionTarget = Target;
        if(motor->MFoc.pos_loop_divider)
        {
            if (times % motor->MFoc.pos_loop_divider == 0) // 降频处理
            {
                motor->MFoc.PositionPID.PID_output = PID_Clu(&motor->MFoc.PositionPID, PositionTarget, motor->AngleMech_acu);
            }
        }
        else motor->MFoc.PositionPID.PID_output = PID_Clu(&motor->MFoc.PositionPID, PositionTarget, motor->AngleMech_acu);// 无降频 直接计算
    case SPEEDCTR:
        if(motor->MFoc.state == SPEEDCTR)SpeedTarget = Target;// 如果是当前模式，则目标值就是设定值
        else SpeedTarget = motor->MFoc.PositionPID.PID_output;// 否则，则是目标值是位置环输出
        if(motor->MFoc.speed_loop_divider)
        {
            if (times % motor->MFoc.speed_loop_divider == 0) // 降频处理
            {
                motor->MFoc.SpeedPID.PID_output = PID_Clu(&motor->MFoc.SpeedPID, SpeedTarget, motor->Speed);
            }
        }
        else motor->MFoc.SpeedPID.PID_output = PID_Clu(&motor->MFoc.SpeedPID, SpeedTarget, motor->Speed);// 无降频 直接计算


    case TORQUECTR:
        if(motor->MFoc.state == TORQUECTR)TorqueTarget = Target;// 如果是当前模式，则目标值就是设定值
        else TorqueTarget = motor->MFoc.SpeedPID.PID_output;// 否则，则是目标值是速度环输出
		
				motor->MFoc.IdPID.PID_output = PID_Clu(&motor->MFoc.IdPID, test, motor->MFoc.Id);
				UdOutput = motor->MFoc.IdPID.PID_output;
		
        motor->MFoc.TorquePID.PID_output = PID_Clu(&motor->MFoc.TorquePID, TorqueTarget, motor->MFoc.Iq);
        UqOutput = motor->MFoc.TorquePID.PID_output;
        break;
    case OPENCTR:
        UqOutput = Target;
        break;
    default:
        break;
    }
    /*位置环*/
    
    /*速度环*/

    /*力矩环*/
		
    _Foc_SetThreePhaseVoltage(UqOutput, UdOutput, motor->SupplyVolt, motor->TimeCount,
                              &motor->MFoc.Ualpha, &motor->MFoc.Ubeta, motor->MFoc.sintheta, motor->MFoc.costheta);
    if (times == 50000)
        times = 0;
}


volatile uint8_t _switch = 0;
float speed_yuzhi = 100000.0f;
void Motor_Run(Motor_Def *motor, uint8_t Command, float Target)
{
    /*角度电流数据获取*/
    __Foc_GetCurrent(&motor->Iabc_raw[0], &motor->Iabc_raw[1], &motor->Iabc_raw[2]);
    __Foc_GetAngle(&motor->AngleMech);
    __Foc_GetAngle_Acu(&motor->AngleMech_acu, motor->AngleMech);
    __Foc_GetSpeed(&motor->Speed, motor->AngleMech, FOCTS);
    /*数据处理*/
    motor->AngleMech = motor->AngleMech - motor->AngleMech_Offset; // 零偏矫正
    if (motor->AngleMech < 0)
        motor->AngleMech += _2PI; // 角度边界处理
#if 1
    motor->AngleMech = _2PI - motor->AngleMech;
    if (motor->AngleMech < 0)
        motor->AngleMech += _2PI;
#endif
    if(motor->Speed >= speed_yuzhi || motor->Speed <= -speed_yuzhi)_switch = 1;
    if(_switch == 0)motor->MFoc.AngleEle = _Foc_GetAngle_ele(motor->AngleMech, motor->pole);
    else motor->MFoc.AngleEle = motor->SMO.theta_hat;
    //motor->MFoc.AngleEle = _Foc_GetAngle_ele(motor->AngleMech, motor->pole);
    motor->MFoc.sintheta = sinf(motor->MFoc.AngleEle); // 三角函数计算
    motor->MFoc.costheta = cosf(motor->MFoc.AngleEle);
    motor->Speed = LowPassFilter_Process(&motor->Speed_LPFilter, motor->Speed);
    motor->Speed = MovingAverageFilter_Process(&motor->Speed_MovingFilter, motor->Speed);
    motor->we = motor->Speed * motor->pole;

    motor->Iabc[0] = motor->Iabc_raw[0] - motor->Iabc_offset[0];
    motor->Iabc[1] = motor->Iabc_raw[1] - motor->Iabc_offset[1];
    motor->Iabc[2] = motor->Iabc_raw[2] - motor->Iabc_offset[2];
    motor->Iabc[0] = _Foc_Current_Standardization(motor->Iabc[0]);
    motor->Iabc[1] = _Foc_Current_Standardization(motor->Iabc[1]);
    motor->Iabc[2] = _Foc_Current_Standardization(motor->Iabc[2]);
    
    /*保存上一时刻Ialpha Ibeta*/
    Ialpha_prv = motor->MFoc.Ialpha;
    Ibeta_prv = motor->MFoc.Ibeta;
    _Foc_clarkParkTransform(motor->Iabc, motor->MFoc.AngleEle, &motor->MFoc.Ialpha, &motor->MFoc.Ibeta,
                            &motor->MFoc.Iq, &motor->MFoc.Id, motor->MFoc.sintheta, motor->MFoc.costheta);
    motor->MFoc.Iq = LowPassFilter_Process(&motor->MFoc.Iq_LPFilter, motor->MFoc.Iq);
	motor->MFoc.Id = LowPassFilter_Process(&motor->MFoc.Id_LPFilter, motor->MFoc.Id);

    /*计算反电动势*/
    float temp  = motor->flux * motor->we;//简化计算
    motor->MFoc.Ealpha = temp * -motor->MFoc.sintheta;
    motor->MFoc.Ebeta = temp * motor->MFoc.costheta;
		
    switch (motor->state)
    {
    case MOTORINIT:
        static uint16_t times = 0;
        static uint8_t InitState = 0;
        times++;
        if (InitState == 0) // 编码器初始化
        {
            if (times == 1)
            {
                motor->MFoc.AngleEle = 0.0f;
                motor->MFoc.sintheta = 0.0f;
                motor->MFoc.costheta = 1.0f;
                _Foc_SetThreePhaseVoltage(0.0f, (motor->SupplyVolt / 2.0f) * 0.3f, motor->SupplyVolt, motor->TimeCount,
                                          &motor->MFoc.Ualpha, &motor->MFoc.Ubeta, motor->MFoc.sintheta, motor->MFoc.costheta);
            }
            else if (times == 20000) // 延时10000个Foc周期，等待电机对齐
            {
                __Foc_GetAngle(&motor->AngleMech_Offset); // 获取电机零偏角
                motor->AngleMech_acu = 0.0f;
            }
            else if (times >= 20000) // 跳转到电流校准
            {
                _Foc_SetThreePhaseVoltage(0.0f, 0.0f, motor->SupplyVolt, motor->TimeCount,
                                          &motor->MFoc.Ualpha, &motor->MFoc.Ubeta, motor->MFoc.sintheta, motor->MFoc.costheta);
                InitState = 1;
            }
        }
        else if (InitState == 1) // 编码器方向校准
        {
					  InitState = 2;
            times = 0;
//            uint8_t state = 1;
//            state = _Foc_EncoderDirIdentify(motor, FOCTS, (motor->SupplyVolt/2)*0.3f);
//            if(state == 0)// 校准完成，跳转到下一阶段
//            {
//                InitState = 2;
//                times = 0;
//            }
        }
        else if (InitState == 2) // 电流校准
        {
            if (times == 10000) // 延时 等待电流稳定(可以选择多次采样取平均值，此处未添加)
            {
                motor->Iabc_offset[0] = motor->Iabc_raw[0];
                motor->Iabc_offset[1] = motor->Iabc_raw[1];
                motor->Iabc_offset[2] = motor->Iabc_raw[2];
                times = 0;
                InitState = 0;
                motor->state = MOTORREADY; // 电机初始化完毕，跳转到电机Ready状态
                motor->MFoc.state = OPENCTR;// 使用开环控制
            }
        }
        break;
    case MOTORIDENTIFY:
        break;
    case MOTORREADY:
        Motor_CTRMode(motor, Target);
        break;
    case MOTORERROR:
        break;
    default:
        break;
    }
}


uint16_t times = 0;// 函数调用次数
uint8_t Identify_state = 0;
float start_angle = 0.0f;
float mid_angle = 0.0f;
float end_angle = 0.0f;
uint16_t loop_t = 0;
uint8_t _Foc_EncoderDirIdentify(Motor_Def *motor, float Ts, float Uq)
{
    loop_t ++;
    // static uint16_t times = 0;// 函数调用次数
    // static uint8_t Identify_state = 0;
    // static float start_angle = 0.0f;
    // static float mid_angle = 0.0f;
    // static float end_angle = 0.0f;
    static int i = 0;
    times ++;
    if(Identify_state == 0)//正转阶段
    {
        if(times*Ts >= 0.5f)// 200ms计时器
        {
            if(i == 0)start_angle = motor->AngleMech + 1.0f;// 获取初始角度 添加1rad偏置，避免电机抖动造成0-6.28的跳动
            times = 0;// 清空计时器
            motor->MFoc.AngleEle = _3PI_2 + _2PI * i / 6.0;
            motor->MFoc.sintheta = sinf(motor->MFoc.AngleEle);
            motor->MFoc.costheta = cosf(motor->MFoc.AngleEle);
            _Foc_SetThreePhaseVoltage(Uq, 0.0f, motor->SupplyVolt, motor->TimeCount,
                                            &motor->MFoc.Ualpha, &motor->MFoc.Ubeta, motor->MFoc.sintheta, motor->MFoc.costheta);
            i++;// 循环递增
            if(i >= 6)// 正转完毕
            {
                mid_angle = motor->AngleMech + 1.0f;// 获取中间角度
                Identify_state = 1;// 跳转到反转阶段
                i = 5;
            }
        }
        return 1;
    }
    else if(Identify_state == 1)// 反转阶段
    {
        if(times*Ts >= 0.5f)// 200ms计时器
        {
            times = 0;// 清空计时器
            motor->MFoc.AngleEle = _3PI_2 + _2PI * i / 6.0;
            motor->MFoc.sintheta = sinf(motor->MFoc.AngleEle);
            motor->MFoc.costheta = cosf(motor->MFoc.AngleEle);
            _Foc_SetThreePhaseVoltage(Uq, 0.0f, motor->SupplyVolt, motor->TimeCount,
                                            &motor->MFoc.Ualpha, &motor->MFoc.Ubeta, motor->MFoc.sintheta, motor->MFoc.costheta);
            i--;// 循环递减
            if(i < 0 )// 反转完毕
            {
                Identify_state = 2;// 跳转到最后阶段
                i = 0;
            }
        }
        return 1;
    }
    else if(Identify_state == 2)
    {
        if (mid_angle < start_angle)
            motor->Encoder_dir = 1; // 编码器反
        else if (mid_angle > start_angle)
            motor->Encoder_dir = 0; // 编码器正
        else
            motor->Encoder_dir = 2; // 编码器无变化
        
        // 判断结束，返回结束信号，复原程序
        Identify_state = 0;
        i = 0;
        times = 0;
        return 0;
    }

}

//     motor->Mode = MOTOR_ENCODER_INIT;
// #if !ENCODER_LESS // 如果有编码器
//     /*校准编码器方向*/
//     float start_angle = 0.0f;
//     __Foc_GetAngle(&start_angle);
//     for (int i = 0; i <= 5; i++)
//     {
//         motor->Angle_ele = _3PI_2 + _2PI * i / 6.0;
//         motor->sintheta = sinf(motor->Angle_ele);
//         motor->costheta = cosf(motor->Angle_ele);
//         Foc_SetThreePhaseVoltage(motor, Uq, 0.0f);
//         __Foc_Delayms(200);
//     }
//     float mid_angle = 0.0f;
//     __Foc_GetAngle(&mid_angle);
//     for (int i = 5; i >= 0; i--)
//     {
//         motor->Angle_ele = _3PI_2 + _2PI * i / 6.0;
//         motor->sintheta = sinf(motor->Angle_ele);
//         motor->costheta = cosf(motor->Angle_ele);
//         Foc_SetThreePhaseVoltage(motor, Uq, 0.0f);
//         __Foc_Delayms(200);
//     }
//     if (mid_angle < start_angle)
//         motor->EncoderDir = 1; // 编码器反
//     else if (mid_angle > start_angle)
//         motor->EncoderDir = 0; // 编码器正
//     else
//         motor->EncoderDir = 2; // 编码器无变化