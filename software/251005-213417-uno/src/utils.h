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
const float d2 = sqrt(d2x*d2x + d2z*d2z);

const float gamma = 47.5 * DEG_TO_RAD;  // Convert degrees to radians
const float H_PI = 1.5707963267948966;

static inline float clampf(float num, float lo, float hi) {
    if (num < lo) return lo;
    if (num > hi) return hi;
    return num;
}
static inline float wrap(float num) {
    if (num < 0) {
        float ccwNum = num + 2*PI;
        if (ccwNum > PI) {
            return PI;
        } else {
            return ccwNum;
        }
    }
    return num;
}



static inline float safe_acos(float t) { return acosf(clampf(t, -1.0f, 1.0f)); }
static inline float safe_asin(float t) { return asinf(clampf(t, -1.0f, 1.0f)); }
static inline float sinTheta(float a, float b, float theta) { return safe_asin((a/b) * sin(theta)); }
static inline float cosTheta(float a, float b, float c) { return safe_acos((a*a + b*b - c*c) / (2*a*b)); }
static inline float cosLawC(float a, float b, float theta) { return sqrt(a*a + b*b - 2*a*b*cos(abs(theta)));}

// Deprecated
// void ik(float x, float y, float z, bool side, float &theta1, float &theta2, float &theta3);


