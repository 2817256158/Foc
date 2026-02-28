#ifndef _TLE5012B_H_
#define _TLE5012B_H_
#include "stm32g4xx_hal.h"

#define GPIO_CS_Pin_Name GPIO_PIN_1
#define GPIO_CS_Pin_Type GPIOB

#define SPI_CS_ENABLE HAL_GPIO_WritePin(GPIO_CS_Pin_Type, GPIO_CS_Pin_Name, GPIO_PIN_RESET)
#define SPI_CS_DISABLE HAL_GPIO_WritePin(GPIO_CS_Pin_Type, GPIO_CS_Pin_Name, GPIO_PIN_SET)

/* SPI command for TLE5012 */
#define READ_STATUS 0x8001      // 8000
#define READ_ANGLE_VALUE 0x8021 // 8020
#define READ_SPEED_VALUE 0x8031 // 8030

#define WRITE_MOD1_VALUE 0x5060 // 0_1010_0_000110_0001
#define MOD1_VALUE 0x0001

#define WRITE_MOD2_VALUE 0x5080 // 0_1010_0_001000_0001
#define MOD2_VALUE 0x0801

#define WRITE_MOD3_VALUE 0x5091 // 0_1010_0_001001_0001
#define MOD3_VALUE 0x0000

#define WRITE_MOD4_VALUE 0x50E0 // 0_1010_0_001110_0001
#define MOD4_VALUE 0x0098       // 9bit 512

#define WRITE_IFAB_VALUE 0x50B1
#define IFAB_VALUE 0x000D
/* Functionality mode */
#define REFERESH_ANGLE 0

// 滑动滤波器结构体
// 添加新值并获取滤波结果
//typedef struct {
//    float buffer[10];    // 滤波缓冲区
//    float sum;           // 当前缓存总和
//    uint8_t head;        // 缓冲区起始位置（最旧数据）
//    uint8_t count;       // 当前有效数据计数
//} SlidingFilter;

void SPI_SendData16(uint16_t SendData);
uint16_t SPI_ReadData16(void);
uint16_t ReadAngle_Raw(void);
uint16_t ReadSpeed(void);
uint16_t ReadValue(uint16_t u16Value);
uint16_t SPIx_ReadWriteByte(uint16_t byte);
uint16_t TlE5012W_Reg(uint16_t Reg_CMD, uint16_t Reg_Data);

float ReadAngle_Rad();
float ReadSpeed_Rad();
#endif
