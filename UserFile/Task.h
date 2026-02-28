#ifndef __TASK_H__
#define __TASK_H__

#include "stm32g4xx_hal.h"

/*vofa数据结构体*/
typedef struct {
    float data[12];      // 7-element float array
    uint8_t footer[4];  // Fixed footer: 0x00,0x00,0x7f,0x80
} UART_DataFrame;


void Sysinit();
void SysRun();
void Test_IF_Control();


#endif
