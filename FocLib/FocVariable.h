#ifndef __FOCVARIABLE_H__
#define __FOCVARIABLE_H__

#define MY_PI 3.1415926f

extern float g_Angle_mech;
extern float g_Angle_ele;
extern float g_Speed; 
extern float g_Ia;
extern float g_Ib;
extern float g_Ic; 
extern float g_Iq; 
extern float g_Id; 
extern float g_Is;
extern float g_Ualpha;
extern float g_Ubeta; 
extern float g_Sector; 

extern unsigned short g_FocTick;
extern unsigned short g_IaRaw;
extern unsigned short g_IbRaw;
extern unsigned short g_IcRaw;

extern float Ialpha_prv;
extern float Ibeta_prv;

extern float g_T[3];

#endif