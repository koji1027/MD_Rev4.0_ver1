#include "driver.hpp"

#include <cmath>

#include "math.hpp"
#include "tim.h"

Driver::Driver(uint16_t max_voltage, uint16_t max_duty)
    : MAX_VOLTAGE(max_voltage), MAX_DUTY(max_duty) {
}

Driver::~Driver() {
}

void Driver::init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // HA
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // LA
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // HB
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // LB
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // HC
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // LC

    float voltages[3] = {999, 999, 999};
    drive(voltages);
}

void Driver::drive(float voltages[3]) {
    int16_t duty[6];
    for (int i = 0; i < 3; i++) {
        voltages[i] += MAX_VOLTAGE / 2;
        if (voltages[i] == 999) {
            duty[2 * i] = 0;
            duty[2 * i + 1] = MAX_DUTY;
        } else {
            if (voltages[i] < 0) {
                voltages[i] = 0;
            } else if (voltages[i] > MAX_VOLTAGE) {
                voltages[i] = MAX_VOLTAGE;
            }
            duty[2 * i] = voltages[i] / MAX_VOLTAGE * MAX_DUTY;
            duty[2 * i + 1] = duty[2 * i];
        }
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty[1]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty[2]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty[3]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty[4]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty[5]);
}

void Driver::driveSquareWave(float voltage, float phase) {
    phase = fmodf(phase, TWO_PI);
    if (phase < 0) {
        phase += TWO_PI;
    }
    float voltages[3] = {999, 999, 999};
    if (phase >= 0 && phase < PI_3) {  // A -> B
        voltages[0] = voltage;
        voltages[1] = 0;
    } else if (phase >= PI_3 && phase < TWO_PI_3) {  // A -> C
        voltages[0] = voltage;
        voltages[2] = 0;
    } else if (phase >= TWO_PI_3 && phase < PI) {  // B -> C
        voltages[1] = voltage;
        voltages[2] = 0;
    } else if (phase >= PI && phase < FOUR_PI_3) {  // B -> A
        voltages[1] = voltage;
        voltages[0] = 0;
    } else if (phase >= FOUR_PI_3 && phase < FIVE_PI_3) {  // C -> A
        voltages[2] = voltage;
        voltages[0] = 0;
    } else {  // C -> B
        voltages[2] = voltage;
        voltages[1] = 0;
    }

    int16_t duty[6];
    for (int i = 0; i < 3; i++) {
        if (voltages[i] == 999) {
            duty[2 * i] = 0;
            duty[2 * i + 1] = MAX_DUTY;
        } else {
            if (voltages[i] < 0) {
                voltages[i] = 0;
            } else if (voltages[i] > MAX_VOLTAGE) {
                voltages[i] = MAX_VOLTAGE;
            }
            duty[2 * i] = voltages[i] / MAX_VOLTAGE * MAX_DUTY;
            duty[2 * i + 1] = duty[2 * i];
        }
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty[1]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty[2]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty[3]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty[4]);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty[5]);
}

void Driver::driveSinWave(float voltage, float phase) {
    float voltage_array[3];
    for (uint8_t i = 0; i < 3; i++) {
        voltage_array[i] = voltage * fastSinf(phase + (float)i * TWO_PI / 3.0f);
    }

    drive(voltage_array);
}

void Driver::freeWheel() {
    float voltages[3] = {999, 999, 999};
    drive(voltages);
}