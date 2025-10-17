#include "filter.h"


//滑动平均滤波
float movingAverageFilter(MovingAverage *filter, float newValue) {
    // 减去最旧的值
    filter->sum -= filter->buffer[filter->index];
    // 添加新值
    filter->buffer[filter->index] = newValue;
    filter->sum += newValue;
    // 更新索引
    filter->index = (filter->index + 1) % WINDOW_SIZE;
    // 返回平均值
    return filter->sum / WINDOW_SIZE;
}

//中值滤波
float medianFilter(float newValue) {
    static float buffer[MEDIAN_WINDOW];
    static uint8_t index = 0;
    float tempBuffer[MEDIAN_WINDOW];
    
    // 存储新值
    buffer[index] = newValue;
    index = (index + 1) % MEDIAN_WINDOW;
    
    // 复制到临时数组
    for(uint8_t i = 0; i < MEDIAN_WINDOW; i++) {
        tempBuffer[i] = buffer[i];
    }
    
    // 排序
    for(uint8_t i = 0; i < MEDIAN_WINDOW-1; i++) {
        for(uint8_t j = i+1; j < MEDIAN_WINDOW; j++) {
            if(tempBuffer[i] > tempBuffer[j]) {
                float temp = tempBuffer[i];
                tempBuffer[i] = tempBuffer[j];
                tempBuffer[j] = temp;
            }
        }
    }
    
    // 返回中值
    return tempBuffer[MEDIAN_WINDOW/2];
}


//一阶低通滤波
float lowPassFilter(float newValue) {
    static float filteredValue = 0;
    filteredValue = ALPHA * newValue + (1 - ALPHA) * filteredValue;
    return filteredValue;
}

//极值平均滤波
float extremeMovingAverage(ExtremeMovingAverage *filter, float newValue) {
    //1.存储新值到滑动窗口
    filter->buffer[filter->index] = newValue;
    filter->index = (filter->index + 1) % WINDOW_SIZE;
    
    //2.找出窗口中的最大值和最小值
    float min = filter->buffer[0];
    float max = filter->buffer[0];
    float sum = 0;
    
    for(uint8_t i = 0; i < WINDOW_SIZE; i++) {
        if(filter->buffer[i] < min) min = filter->buffer[i];
        if(filter->buffer[i] > max) max = filter->buffer[i];
        sum += filter->buffer[i];
    }
    
    //3.去除最大最小值后计算平均值
    return (sum - min - max) / (WINDOW_SIZE - 3);
}

// // 初始化
// ExtremeMovingAverage emaFilter = {0};

// // 使用示例
// float rawValue = ADS1256_GetVolt(channel, mode);
// float filteredValue = extremeMovingAverage(&emaFilter, rawValue);

//加权极值平均滤波
float weightedExtremeMovingAverage(float newValue) {
    static float buffer[WINDOW_SIZE];
    static uint8_t index = 0;
    
    // 存储新值
    buffer[index] = newValue;
    index = (index + 1) % WINDOW_SIZE;
    
    // 找出最大最小值
    float min = buffer[0], max = buffer[0];
    uint8_t minIndex = 0, maxIndex = 0;
    
    for(uint8_t i = 1; i < WINDOW_SIZE; i++) {
        if(buffer[i] < min) { min = buffer[i]; minIndex = i; }
        if(buffer[i] > max) { max = buffer[i]; maxIndex = i; }
    }
    
    // 计算加权平均（越新的数据权重越高）
    float sum = 0;
    float weightSum = 0;
    
    for(uint8_t i = 0; i < WINDOW_SIZE; i++) {
        if(i != minIndex && i != maxIndex) {
            float weight = (i + 1); // 线性权重
            sum += buffer[i] * weight;
            weightSum += weight;
        }
    }
    
    return sum / weightSum;
}





