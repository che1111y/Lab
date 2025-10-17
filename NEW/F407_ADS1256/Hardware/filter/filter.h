#ifndef _FILTER_H_
#define _FILTER_H_

#include "stdio.h"
#include "main.h"

#define WINDOW_SIZE 8  // 建议窗口大小至少为3
#define MEDIAN_WINDOW 5
#define ALPHA 0.2f  // 滤波系数 (0 < ALPHA < 1), 越小滤波效果越强

typedef struct {
    float buffer[WINDOW_SIZE];
    uint8_t index;
} ExtremeMovingAverage;

typedef struct {
    float buffer[WINDOW_SIZE];
    uint8_t index;
    float sum;
} MovingAverage;


float movingAverageFilter(MovingAverage *filter, float newValue);
float medianFilter(float newValue);
float lowPassFilter(float newValue);
float extremeMovingAverage(ExtremeMovingAverage *filter, float newValue);
float weightedExtremeMovingAverage(float newValue);

#endif
