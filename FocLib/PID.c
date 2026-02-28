#include "PID.h"

void PID_Clear(PID_Struct *object)
{
    object->P_output = 0.0f;
    object->I_output = 0.0f;
    object->D_output = 0.0f;

    object->error_acu = 0.0f;
    object->error_prv = 0.0f;
}

float PID_Clu(PID_Struct *object, float expect, float feedback)
{
    object->error = expect - feedback;
    
    /*检查是否存在误差限幅*/
    if(object->error_limit)
    {
        object->error = (object->error > object->error_limit ? object->error_limit : object->error);
        object->error = (object->error < -object->error_limit ? -object->error_limit : object->error);
    }

    /*检查是否存在积分分离*/
    if(object->acu_div)
    {
        if(object->error < object->acu_div && object->error > -object->acu_div)object->error_acu += object->error;
        else object->error_acu = 0.0f;
    }
    else object->error_acu += object->error;

    /*检查是否存在积分限幅*/
    if(object->acu_limit)
    {
        object->error_acu = (object->error_acu > object->acu_limit ? object->acu_limit : object->error_acu);
        object->error_acu = (object->error_acu < -object->acu_limit ? -object->acu_limit : object->error_acu);
    }

    object->P_output = object->Kp * object->error;
    object->I_output = object->Ki * object->error_acu;
    object->D_output = object->Kd * (object->error - object->error_prv);
    object->PID_output = object->P_output + object->I_output + object->D_output;
    
    /*检查是否存在比例限幅*/
    if(object->P_limit)
    {

    }

    /*检查是否存在输出限制*/
    if(object->outputLimit != 0.0f)
    {
        object->PID_output  = (object->PID_output > object->outputLimit ? object->outputLimit : object->PID_output);
        object->PID_output  = (object->PID_output < -object->outputLimit ? -object->outputLimit : object->PID_output);
    }

    /*误差记录更新*/
    object->error_prv = object->error;

    return object->PID_output;
}