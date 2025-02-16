#include "math.hpp"

static float SIN_TABLE[SIN_COS_TABLE_SIZE];
static float COS_TABLE[SIN_COS_TABLE_SIZE];

void fastSinfInit() {
    for (uint16_t i = 0; i < SIN_COS_TABLE_SIZE; ++i) {
        SIN_TABLE[i] = sinf(i * STEP);
    }
}

void fastCosfInit() {
    for (uint16_t i = 0; i < SIN_COS_TABLE_SIZE; ++i) {
        COS_TABLE[i] = cosf(i * STEP);
    }
}

float fastSinf(float x) {
    x = fmodf(x, TWO_PI);
    if (x < 0) {
        x += TWO_PI;
    }
    return SIN_TABLE[(uint16_t)(x / STEP)];
}

float fastCosf(float x) {
    x = fmodf(x, TWO_PI);
    if (x < 0) {
        x += TWO_PI;
    }
    return COS_TABLE[(uint16_t)(x / STEP)];
}