#ifndef _PID_H_
#define _PID_H_

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float error_prv;
    float error_acu;
    
    float P_limit;//比例限幅
    float error_limit;//误差限幅
    float acu_limit;//积分限幅
    float acu_div;//积分分离
    float outputLimit;//输出限幅

    float P_output;
    float I_output;
    float D_output;
    float PID_output;

}PID_Struct;

void PID_Clear(PID_Struct *object);
float PID_Clu(PID_Struct *object, float expect, float feedback);
//void PID_CluNomal(PID_Struct *object, float expect, float feedback);

#endif