#ifndef __FOCINTERFACE_H__
#define __FOCINTERFACE_H__

#include "stm32g4xx_hal.h"

void __FocInterface_Init(void);
void __Foc_Delayms(uint16_t ms);
void __Foc_SetPwm(float Tcmp1, float Tcmp2, float Tcmp3);
void __Foc_GetCurrent(uint16_t *Ia, uint16_t *Ib, uint16_t *Ic);
void __Foc_GetAngle(float *angle);
void __Foc_GetAngle_Acu(float *Angle_Acu, float Angle_Mech);
void __Foc_GetSpeed(float *Speed, float Angle, float Ts);
void __Foc_GetVbus(float *Vbus);
void __Foc_GetTemperature(float *Temperature);
void __Foc_GetCommand(uint8_t Command, float Param);/*通过16进制编码实现命令控制 @param1:命令类型 @param2:命令参数*/
#endif