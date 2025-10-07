#pragma once

#include <Arduino.h>

const float L1 = 50.7;
const float L2 = 10;
const float L3 = 100.21;
const float L4 = 50.42;
const float L5 = 100;
const float L6 = 134.16;
const float L7 = 70.94;
const float L8 = 52.5;
const float L9 = 60;
const float L10 = 24.5;

const float d1 = 86.79;
const float d2z = 25.5;
const float d2x = 19.3;

const float gamma1 = 85 * DEG_TO_RAD;  // Convert degrees to radians
const float H_PI = 1.5707963267948966;

void ik(float x, float y, float z, bool side, float &theta1, float &theta2, float &theta3);

// utils.h
static inline float clampf(float num, float lo, float hi) {
    if (num < lo) return lo;
    if (num > hi) return hi;
    return num;
}
float clamp(float num);
static inline float safe_acos(float t) { return acosf(clampf(t, -1.0f, 1.0f)); }
static inline float safe_asin(float t) { return asinf(clampf(t, -1.0f, 1.0f)); }

