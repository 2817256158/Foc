#ifndef __M24C64_H_
#define __M24C64_H_

#include "stm32g4xx_hal.h"  // 根据实际MCU型号修改

// 函数声明
void I2C_Init(void);
uint8_t M24C64_WriteByte(uint16_t addr, uint8_t data);
uint8_t M24C64_ReadByte(uint16_t addr, uint8_t *data);

#endif /*  */
