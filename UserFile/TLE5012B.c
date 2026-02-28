#include "TLE5012B.h"

#define RANGE 32768             
#define ZERO_OFFSET 0x1EC8 //实机测试得到
#define EN_DIR 1
#define _2PI 6.28318f

extern SPI_HandleTypeDef hspi1;

uint16_t g_u16AngleOffset = ZERO_OFFSET;  // 角度校准偏移量定义

uint16_t SPIx_ReadWriteByte(uint16_t byte)
{

        uint16_t read_value = 0;
        
        HAL_SPI_Transmit( &hspi1, (uint8_t *)(&byte), sizeof(byte)/sizeof(uint16_t), 0xff );
        
        HAL_SPI_Transmit( &hspi1, (uint8_t *)(&read_value), sizeof(read_value)/sizeof(uint16_t), 0xff );
        return read_value;
}
 
// 返回0-32767
uint16_t ReadAngle_Raw(void)
{
        return ((ReadValue(READ_ANGLE_VALUE) - RANGE));// * 360.0 / 0x10000 );
}
 

uint16_t ReadSpeed(void)
{
        return ReadValue(READ_SPEED_VALUE);
}
 
 
uint16_t ReadValue(uint16_t u16RegValue)
{
        uint16_t u16Data;
 
        SPI_CS_ENABLE;
        
        HAL_SPI_Transmit( &hspi1, (uint8_t *)(&u16RegValue), sizeof(u16RegValue)/sizeof(uint16_t), 0xff );
        HAL_SPI_Receive( &hspi1,(uint8_t *)(&u16Data), sizeof(u16Data)/sizeof(uint16_t), 0xff );
        
        SPI_CS_DISABLE;

        
        return (((u16Data) & 0x7FFF)<<1);
}


float ReadAngle_Rad()
{
        uint16_t RawAngle = 0x00;
        RawAngle = ReadAngle_Raw();
        return (RawAngle*_2PI / RANGE);
}

#define T_upd 2.0f*42.7f  //更新时间
float ReadSpeed_Rad()
{
        static float Speed_Prv = 0.0f;
        uint16_t Speed_Raw = ReadValue(READ_SPEED_VALUE);
        Speed_Raw = (Speed_Raw & 0x7FFF);
        uint16_t sign = Speed_Raw & 0x4000;
        if(sign == 0x4000)
        {
                Speed_Raw = Speed_Raw | 0x8000;
        }
        Speed_Raw = Speed_Raw * 1000;
        int16_t Speed_t = (int16_t)Speed_Raw;
        float Speed_ret = (float)(Speed_t * 2.0f * 3.14f) / 0x7FFF; // rad/s
        Speed_ret = Speed_ret / T_upd;
        //Speed_ret = Speed_ret * 0.2f + Speed_Prv * 0.8f;

        //Speed_Prv =Speed_ret;

        #if EN_DIR == 1
                return Speed_ret*-1.0f;
        #endif

        return Speed_ret;
}
