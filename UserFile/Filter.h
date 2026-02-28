#ifndef _FILTER_H_
#define _FILTER_H_

#include "stm32g4xx_hal.h"

typedef struct {
    float *buffer;          // 数据缓存
    uint16_t buffer_size;   // 缓存大小
    uint16_t filter_length; // 滤波窗口长度
    uint16_t current_index; // 当前索引
    uint16_t sample_count;  // 已采样数量
    float sum;              // 当前和（用于优化计算）
} MovingAverageFilter_t;

typedef struct {
    float alpha;           // 滤波系数 (0-1)
    float prev_output;     // 上一次的输出值
    uint8_t is_first_sample; // 是否是第一个样本
} LowPassFilter_t;

// 滑动窗口滤波函数声明
void MovingAverageFilter_Init(MovingAverageFilter_t *filter, float *buffer, 
                             uint16_t buffer_size, uint16_t filter_length);
float MovingAverageFilter_Process(MovingAverageFilter_t *filter, float new_value);
void MovingAverageFilter_Reset(MovingAverageFilter_t *filter);
// 低通滤波函数声明
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha);
float LowPassFilter_Process(LowPassFilter_t *filter, float new_value);
void LowPassFilter_Reset(LowPassFilter_t *filter);
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha);
float LowPassFilter_GetAlpha(const LowPassFilter_t *filter);
float LowPass_Simple(float new_value, float old_value, float alpha);

#endif