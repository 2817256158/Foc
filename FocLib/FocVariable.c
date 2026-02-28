#include "FocVariable.h"


float g_Ualpha = 0.0f;
float g_Ubeta = 0.0f;
float g_Sector = 0.0f;

float g_Angle_mech = 0.0f;
float g_Angle_ele = 0.0f;
float g_Speed = 0.0f;
float g_Ia = 0.0f;
float g_Ib = 0.0f;
float g_Ic = 0.0f;
float g_Iq = 0.0f;
float g_Id = 0.0f;
float g_Is = 0.0f;


unsigned short g_FocTick = 0;
unsigned short g_IaRaw = 0;
unsigned short g_IbRaw = 0;
unsigned short g_IcRaw = 0;

float Ialpha_prv = 0.0f;
float Ibeta_prv = 0.0f;

float g_T[3] = {0};
