#ifndef CURRENT_SENSOR_HPP
#define CURRENT_SENSOR_HPP

#include "stm32g431xx.h"

#define CURRENT_SENSE_CALIBRATION_SAMPLE_NUM 100
#define CURRENT_SENSE_MOVING_AVERAGE_SAMPLE_NUM 1
#define CURRENT_SENSE_LPF_ALPHA 1.0f

class CurrentSensor {
   private:
    const uint16_t ADC_RES;
    const float ADC_REF_VOLTAGE;
    const uint8_t AMP_GAIN;
    const float SHUNT_RESISTANCE;

    uint16_t adcRawVal[3];
    int32_t adcVal[3];
    int32_t adcValBuf[3][CURRENT_SENSE_MOVING_AVERAGE_SAMPLE_NUM];
    uint8_t adcValIndex[3];
    uint16_t adcValOffset[3];
    float current[3];          // mA
    float prevCurrent[3];      // mA
    float currentFiltered[3];  // mA

    void calibrate();
    void update(uint8_t phase);

   public:
    CurrentSensor(uint16_t adcRes, float adcRefVoltage, uint8_t ampGain, float shuntResistance);
    ~CurrentSensor();

    void init();
    uint16_t getAdcRawVal(uint8_t type, uint8_t phase);
    int32_t getAdcVal(uint8_t phase);
    float getCurrent(uint8_t phase);
};

#endif  // CURRENT_SENSOR_HPP