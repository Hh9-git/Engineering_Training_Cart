
#ifndef ALG_FASTMATH_H
#define ALG_FASTMATH_H

#include "main.h"
#define FAST_MATH_TABLE_SIZE 512

#define PI 3.14159265358979f
#define PI_HALF 1.5707963267948966192313f

// 快速正弦
float fast_sin(float radians);
// 快速余弦
float fast_cos(float radians);
// 快速平方根，不如sqrtf
bool fast_sqrt(float in, float *pOut);
// 快速反正切, 比atan快不了多少
float fast_atan(float x);
// 快速反正切2
bool fast_atan2(float y, float x, float *result);


#endif //ALG_FASTMATH_H