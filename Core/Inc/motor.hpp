#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <memory>

#include "driver.hpp"
#include "enc.hpp"
#include "math.hpp"

#define CALIBRATION_SAMPLE_NUM 5
#define ADVANCED_ANGLE PI * 10.0f / 24.0f

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

    void motorCalibrate();
    void calcElecAngle();
    void calcPhase(bool turn);

   public:
    Motor(uint16_t encRes, float batteryVoltage, uint16_t maxDuty, uint8_t motorPolesNum);
    ~Motor();

    std::unique_ptr<Encoder> enc;
    std::unique_ptr<Driver> driver;

    void init();
    void release();
    void invertTurn();
    float getElecAngle();
    float getPhase();
};

#endif