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

void Driver::driveSquareWave(float voltage, uint16_t step) {
    step %= 6;

    uint16_t duty = voltage / MAX_VOLTAGE * MAX_DUTY;

    switch (step) {
        case 0:  // A -> B
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
            break;

        case 1:  // A -> C
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            break;

        case 2:  // B -> C
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            break;

        case 3:  // B -> A
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
            break;

        case 4:  // C -> A
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
            break;

        case 5:  // C -> B
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
            break;

        default:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
    }
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