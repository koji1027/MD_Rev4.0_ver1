#include "current_sensor.hpp"

#include <stdio.h>

#include "adc.h"
#include "usart.h"

CurrentSensor::CurrentSensor(uint16_t adcRes, float adcRefVoltage, uint8_t ampGain, float shuntResistance)
    : ADC_RES(adcRes), ADC_REF_VOLTAGE(adcRefVoltage), AMP_GAIN(ampGain), SHUNT_RESISTANCE(shuntResistance) {
    for (int i = 0; i < 3; i++) {
        adcRawVal[i] = 0.0f;
        adcVal[i] = 0.0f;
        adcValOffset[i] = 0.0f;
        current[i] = 0.0f;
    }
}

CurrentSensor::~CurrentSensor() {}

void CurrentSensor::init() {
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcRawVal, 3);
    HAL_Delay(100);
    calibrate();
}

void CurrentSensor::calibrate() {
    uint32_t adcValSum[CURRENT_SENSE_CALIBRATION_SAMPLE_NUM] = {0};
    for (int i = 0; i < CURRENT_SENSE_CALIBRATION_SAMPLE_NUM; i++) {
        for (int j = 0; j < 3; j++) {
            adcValSum[i] += adcRawVal[j];
        }
    }
    int64_t adcValOffsetSum = 0;
    for (int i = 0; i < CURRENT_SENSE_CALIBRATION_SAMPLE_NUM; i++) {
        adcValOffsetSum += adcValSum[i];
    }
    for (int i = 0; i < 3; i++) {
        adcValOffset[i] = adcValOffsetSum / CURRENT_SENSE_CALIBRATION_SAMPLE_NUM / 3;
    }
}

void CurrentSensor::update(uint8_t phase) {
    int32_t tempAdcVal = adcRawVal[phase] - adcValOffset[phase];

    adcValBuf[phase][adcValIndex[phase]] = tempAdcVal;
    adcValIndex[phase] = (adcValIndex[phase] + 1) % CURRENT_SENSE_MOVING_AVERAGE_SAMPLE_NUM;

    int32_t adcValSum = 0;
    for (int i = 0; i < CURRENT_SENSE_MOVING_AVERAGE_SAMPLE_NUM; i++) {
        adcValSum += adcValBuf[phase][i];
    }
    adcVal[phase] = adcValSum / CURRENT_SENSE_MOVING_AVERAGE_SAMPLE_NUM;

    current[phase] = (float)adcVal[phase] / (float)ADC_RES * (float)ADC_REF_VOLTAGE / (float)AMP_GAIN / (float)SHUNT_RESISTANCE * 1000.0f;

    currentFiltered[phase] = current[phase] * CURRENT_SENSE_LPF_ALPHA + prevCurrent[phase] * (1.0f - CURRENT_SENSE_LPF_ALPHA);
    prevCurrent[phase] = currentFiltered[phase];
}

uint16_t CurrentSensor::getAdcRawVal(uint8_t type, uint8_t phase) {
    if (type == 0) {
        return adcRawVal[phase];
    } else {
        return adcValOffset[phase];
    }
}

int32_t CurrentSensor::getAdcVal(uint8_t phase) {
    return adcVal[phase];
}

float CurrentSensor::getCurrent(uint8_t phase) {
    update(phase);
    return current[phase];
}