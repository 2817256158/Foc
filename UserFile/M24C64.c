#include "M24C64.h"


// 引脚定义
#define SCL_PIN         GPIO_PIN_8
#define SDA_PIN         GPIO_PIN_9
#define I2C_PORT        GPIOC

// 设备地址（根据A0/A1/A2引脚调整）
#define M24C64_ADDR     0xA0  // 7位地址0x50左移1位，读写位自动添加

// 延时函数（根据实际主频调整，确保I2C时序正确）
static void I2C_Delay(uint16_t us)
{
    uint16_t i, j;
    for(i = 0; i < us; i++)
        for(j = 0; j < 10; j++);
}

// 初始化模拟I2C
void I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIOC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // 配置SCL和SDA为推挽输出
    GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
    
    // 初始化为高电平（I2C总线空闲状态）
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);
    I2C_Delay(10);
}

// 设置SDA为输出模式
static void SDA_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

// 设置SDA为输入模式
static void SDA_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

// 产生I2C起始信号
static void I2C_Start(void)
{
    SDA_Output();
    
    // 起始条件：SCL高电平时，SDA从高变低
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay(5);
    
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay(5);
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);  // 拉低SCL，准备发送数据
}

// 产生I2C停止信号
static void I2C_Stop(void)
{
    SDA_Output();
    
    // 停止条件：SCL高电平时，SDA从低变高
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET);
    I2C_Delay(5);
    
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);
    I2C_Delay(5);
    HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);
    I2C_Delay(5);
}

// 发送应答信号（0：应答，1：非应答）
static void I2C_SendAck(uint8_t ack)
{
    SDA_Output();
    
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
    if(ack)
        HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);  // 非应答
    else
        HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET); // 应答
    
    I2C_Delay(5);
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);      // 产生时钟
    I2C_Delay(5);
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
}

// 接收应答信号
static uint8_t I2C_ReceiveAck(void)
{
    uint8_t ack;
    
    SDA_Input();
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
    I2C_Delay(5);
    
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);      // 产生时钟
    I2C_Delay(5);
    
    // 读取应答信号
    if(HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN) == GPIO_PIN_SET)
        ack = 1;  // 非应答
    else
        ack = 0;  // 应答
    
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
    return ack;
}

// 发送一个字节
static void I2C_SendByte(uint8_t data)
{
    uint8_t i;
    SDA_Output();
    
    for(i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);  // 拉低SCL，准备发送数据位
        
        // 发送高位在前
        if(data & 0x80)
            HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(I2C_PORT, SDA_PIN, GPIO_PIN_RESET);
        
        data <<= 1;
        I2C_Delay(5);
        HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);      // 拉高SCL，从机读取数据位
        I2C_Delay(5);
    }
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
}

// 接收一个字节
static uint8_t I2C_ReceiveByte(void)
{
    uint8_t i, data = 0;
    SDA_Input();
    
    for(i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
        I2C_Delay(5);
        HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_SET);      // 拉高SCL，读取数据位
        
        data <<= 1;
        if(HAL_GPIO_ReadPin(I2C_PORT, SDA_PIN) == GPIO_PIN_SET)
            data |= 0x01;
        
        I2C_Delay(5);
    }
    HAL_GPIO_WritePin(I2C_PORT, SCL_PIN, GPIO_PIN_RESET);
    return data;
}

/**
 * @brief  向M24C64指定地址写入一个字节
 * @param  addr: 写入地址(0~8191)
 * @param  data: 要写入的字节
 * @retval 0:成功 1:失败
 */
uint8_t M24C64_WriteByte(uint16_t addr, uint8_t data)
{
    I2C_Start();
    
    // 发送设备地址+写命令
    I2C_SendByte(M24C64_ADDR);
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;  // 设备无应答
    }
    
    // 发送高8位地址
    I2C_SendByte((uint8_t)(addr >> 8));
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;
    }
    
    // 发送低8位地址
    I2C_SendByte((uint8_t)addr);
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;
    }
    
    // 发送数据
    I2C_SendByte(data);
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;
    }
    
    I2C_Stop();
    HAL_Delay(5);  // 等待写入完成
    return 0;
}

/**
 * @brief  从M24C64指定地址读取一个字节
 * @param  addr: 读取地址(0~8191)
 * @param  data: 存储读取数据的指针
 * @retval 0:成功 1:失败
 */
uint8_t M24C64_ReadByte(uint16_t addr, uint8_t *data)
{
    if(data == NULL) return 1;
    
    I2C_Start();
    
    // 发送设备地址+写命令（用于设置读取地址）
    I2C_SendByte(M24C64_ADDR);
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;
    }
    
    // 发送高8位地址
    I2C_SendByte((uint8_t)(addr >> 8));
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;
    }
    
    // 发送低8位地址
    I2C_SendByte((uint8_t)addr);
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;
    }
    
    // 重新发送起始信号，切换为读模式
    I2C_Start();
    I2C_SendByte(M24C64_ADDR | 0x01);  // 读命令
    if(I2C_ReceiveAck() != 0)
    {
        I2C_Stop();
        return 1;
    }
    
    // 读取数据
    *data = I2C_ReceiveByte();
    I2C_SendAck(1);  // 发送非应答，结束读取
    I2C_Stop();
    
    return 0;
}
