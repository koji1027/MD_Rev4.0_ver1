#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <stdio.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "current_sensor.hpp"
#include "driver.hpp"
#include "enc.hpp"
#include "math.hpp"
#include "tim.h"
#include "usart.h"

#define ADVANCED_ANGLE PI
#define RPM_MOVING_AVERAGE_SAMPLE_NUM 5
#define MOTOR_CALIBRATION_SAMPLE_NUM 5

class Motor {
   private:
    const uint16_t ENC_RES;
    const float BATTERY_VOLTAGE;
    const uint16_t MAX_DUTY;
    const uint8_t MOTOR_POLES_NUM;
    const float ENC_VAL_PER_ELEC_ANGLE;

    float elecAngle;
    float phase;
    bool turn;
    float rpm;

    void calcElecAngle();
    void calcPhase(bool turn);
    void motorCalibrate();

   public:
    Motor(uint16_t encRes, float batteryVoltage, uint16_t maxDuty, uint8_t motorPolesNum, uint16_t currentAdcRes, float currentAdcRefVoltage, uint8_t currentAmpGain, float currentShuntResistance);
    ~Motor();

    std::unique_ptr<Encoder> enc;
    std::unique_ptr<Driver> driver;
    std::unique_ptr<CurrentSensor> currentSensor;

    void init();
    void timerStart();
    void release();
    void setTurn(bool turn);
    float getElecAngle();
    float getPhase();
    void calcRpm(uint64_t time_us);  // us
    float getRpm();
};

#endif