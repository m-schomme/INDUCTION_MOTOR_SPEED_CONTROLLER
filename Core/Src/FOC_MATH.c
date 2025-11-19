/*
 * FOC_MATH.c
 *
 *  Created on: Oct 30, 2025
 *      Author: maryr
 */
#include "arm_math.h"
#include "math.h"
#include "FOC_MATH.h"

// normalizes angle to [0, 2PI) by +/- 2PI
void norm_angle_rad(float *theta) {
    while (*theta < 0) *theta += TWO_PI;
    while (*theta >= TWO_PI) *theta -= TWO_PI;
}

// fast sine and cosine using CMSIS-DSP library
float fast_sin(float theta) {
    return arm_sin_f32(theta);
}

float fast_cos(float theta) {
    return arm_cos_f32(theta);
}

// call together
void pre_calc_sin_cos(float theta, float *sin_theta, float *cos_theta) {
    *sin_theta = fast_sin(theta);
    *cos_theta = fast_cos(theta);
}

// fast combined clarke + park transform
void clarke_park_transform(float ia, float ib, float sin_theta, float cos_theta, float *id, float *iq) {
    float i_alpha = ia;
    float i_beta = ONE_BY_SQRT3 *ia + TWO_BY_SQRT3 * ib;

    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = i_alpha* -sin_theta + i_beta * cos_theta;
}

// inverse park transform (dq to αβ)
void inverse_park_transform(float vd, float vq, float sin_theta, float cos_theta, float *valpha, float *vbeta) {
    *valpha = vd * cos_theta - vq * sin_theta;
    *vbeta = vd * sin_theta + vq * cos_theta;
}

// inverse clarke transform  (αβ0 to abc)
void inverse_clarke_transform(float valpha, float vbeta, float *va, float *vb, float *vc) {
    *va = valpha;
    *vb = -0.5f * valpha + SQRT3_BY_TWO * vbeta;
    *vc = -0.5f * valpha - SQRT3_BY_TWO * vbeta;
}

// space vector pwm
void svpwm(float valpha, float vbeta, float vbus, uint32_t pwm_period,
            uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w)
{
    // 1. normalize voltages to DC bus
    if (vbus == 0.0f) {
        if (pwm_u) *pwm_u = 0;
        if (pwm_v) *pwm_v = 0;
        if (pwm_w) *pwm_w = 0;
        return;
    }

    float alpha = valpha / vbus;
    float beta  = vbeta  / vbus;

    // 2. determine sector
    uint8_t sector;
    if (beta >= 0.0f) {
        if (alpha >= 0.0f)
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 2 : 1;
        else
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 3 : 2;
    } else {
        if (alpha >= 0.0f)
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 5 : 6;
        else
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 4 : 5;
    }

    // 3. calcualte t1 and t2
    float pwm_period_f = (float)pwm_period;
    float T1 = 0.0f, T2 = 0.0f;

    switch (sector) {
        case 1:
            T1 = (alpha - ONE_BY_SQRT3 * beta) * pwm_period_f;
            T2 = (TWO_BY_SQRT3 * beta) * pwm_period_f;
            break;
        case 2:
            T1 = (alpha + ONE_BY_SQRT3 * beta) * pwm_period_f;
            T2 = (-alpha + ONE_BY_SQRT3 * beta) * pwm_period_f;
            break;
        case 3:
            T1 = (TWO_BY_SQRT3 * beta) * pwm_period_f;
            T2 = (-alpha - ONE_BY_SQRT3 * beta) * pwm_period_f;
            break;
        case 4:
            T1 = (-alpha + ONE_BY_SQRT3 * beta) * pwm_period_f;
            T2 = (-TWO_BY_SQRT3 * beta) * pwm_period_f;
            break;
        case 5:
            T1 = (-alpha - ONE_BY_SQRT3 * beta) * pwm_period_f;
            T2 = (alpha - ONE_BY_SQRT3 * beta) * pwm_period_f;
            break;
        case 6:
            T1 = (-TWO_BY_SQRT3 * beta) * pwm_period_f;
            T2 = (alpha + ONE_BY_SQRT3 * beta) * pwm_period_f;
            break;
        default:
            T1 = T2 = 0.0f;
            break;
    }

    // 4. phase timings
    float T0 = pwm_period_f - (T1 + T2);
    float TA, TB, TC;

    switch (sector) {
        case 1: TA = 0.5f * T0; TB = TA + T1; TC = TB + T2; break;
        case 2: TB = 0.5f * T0; TA = TB + T2; TC = TA + T1; break;
        case 3: TB = 0.5f * T0; TC = TB + T1; TA = TC + T2; break;
        case 4: TC = 0.5f * T0; TB = TC + T2; TA = TB + T1; break;
        case 5: TC = 0.5f * T0; TA = TC + T1; TB = TA + T2; break;
        case 6: TA = 0.5f * T0; TC = TA + T2; TB = TC + T1; break;
        default: TA = TB = TC = pwm_period_f * 0.5f; break;
    }

    // 5. clamp and assign
    if (TA < 0) TA = 0; if (TA > pwm_period_f) TA = pwm_period_f;
    if (TB < 0) TB = 0; if (TB > pwm_period_f) TB = pwm_period_f;
    if (TC < 0) TC = 0; if (TC > pwm_period_f) TC = pwm_period_f;

    *pwm_u = (uint32_t)(TA);
    *pwm_v = (uint32_t)(TB);
    *pwm_w = (uint32_t)(TC);
}






