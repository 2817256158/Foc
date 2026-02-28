#include "Filter.h"
#include <String.h>

/**
 * @brief 初始化滑动窗口滤波器
 * @param filter: 滤波器结构体指针
 * @param buffer: 外部提供的缓存数组
 * @param buffer_size: 缓存大小
 * @param filter_length: 滤波窗口长度
 */
void MovingAverageFilter_Init(MovingAverageFilter_t *filter, float *buffer, 
                             uint16_t buffer_size, uint16_t filter_length)
{
    if (filter == NULL || buffer == NULL || 
        filter_length == 0 || filter_length > buffer_size) {
        return;
    }
    
    filter->buffer = buffer;
    filter->buffer_size = buffer_size;
    filter->filter_length = filter_length;
    filter->current_index = 0;
    filter->sample_count = 0;
    filter->sum = 0.0f;
    
    // 清空缓存
    memset(buffer, 0, buffer_size * sizeof(float));
}

/**
 * @brief 滑动窗口滤波处理（优化版本）
 * @param filter: 滤波器结构体指针
 * @param new_value: 新输入的值
 * @return 滤波后的值
 */
float MovingAverageFilter_Process(MovingAverageFilter_t *filter, float new_value)
{
    if (filter == NULL || filter->buffer == NULL) {
        return new_value;
    }
    
    // 如果采样数不足，直接累加
    if (filter->sample_count < filter->filter_length) {
        filter->buffer[filter->current_index] = new_value;
        filter->sum += new_value;
        filter->sample_count++;
    } else {
        // 减去最旧的值，加上最新的值
        filter->sum = filter->sum - filter->buffer[filter->current_index] + new_value;
        filter->buffer[filter->current_index] = new_value;
    }
    
    // 更新索引
    filter->current_index = (filter->current_index + 1) % filter->filter_length;
    
    // 计算平均值
    if (filter->sample_count > 0) {
        uint16_t valid_length = (filter->sample_count < filter->filter_length) ? 
                               filter->sample_count : filter->filter_length;
        return filter->sum / valid_length;
    }
    
    return new_value;
}

/**
 * @brief 重置滤波器
 * @param filter: 滤波器结构体指针
 */
void MovingAverageFilter_Reset(MovingAverageFilter_t *filter)
{
    if (filter == NULL || filter->buffer == NULL) {
        return;
    }
    
    filter->current_index = 0;
    filter->sample_count = 0;
    filter->sum = 0.0f;
    memset(filter->buffer, 0, filter->buffer_size * sizeof(float));
}

/**
 * @brief 初始化低通滤波器
 * @param filter: 低通滤波器结构体指针
 * @param alpha: 滤波系数 (0-1)，越小滤波效果越强
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL) {
        return;
    }
    
    // 限制alpha在0-1之间
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    filter->alpha = alpha;
    filter->prev_output = 0.0f;
    filter->is_first_sample = 1;
}

/**
 * @brief 低通滤波处理
 * @param filter: 低通滤波器结构体指针
 * @param new_value: 新输入的值
 * @return 滤波后的值
 */
float LowPassFilter_Process(LowPassFilter_t *filter, float new_value)
{
    if (filter == NULL) {
        return new_value;
    }
    
    if (filter->is_first_sample) {
        // 第一个样本，直接输出
        filter->prev_output = new_value;
        filter->is_first_sample = 0;
        return new_value;
    }
    
    // 低通滤波公式: output = alpha * new_value + (1 - alpha) * prev_output
    float output = filter->alpha * new_value + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = output;
    
    return output;
}

/**
 * @brief 重置低通滤波器
 * @param filter: 低通滤波器结构体指针
 */
void LowPassFilter_Reset(LowPassFilter_t *filter)
{
    if (filter == NULL) {
        return;
    }
    
    filter->prev_output = 0.0f;
    filter->is_first_sample = 1;
}

/**
 * @brief 设置低通滤波器系数
 * @param filter: 低通滤波器结构体指针
 * @param alpha: 新的滤波系数 (0-1)
 */
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL) {
        return;
    }
    
    // 限制alpha在0-1之间
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    filter->alpha = alpha;
}

/**
 * @brief 获取低通滤波器系数
 * @param filter: 低通滤波器结构体指针
 * @return 当前的滤波系数
 */
float LowPassFilter_GetAlpha(const LowPassFilter_t *filter)
{
    if (filter == NULL) {
        return 1.0f;
    }
    
    return filter->alpha;
}

float LowPass_Simple(float new_value, float old_value, float alpha)
{
    return (new_value*alpha + old_value*(1-alpha));
}