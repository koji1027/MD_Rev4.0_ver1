#include "motor.hpp"

Motor::Motor(uint16_t encRes, float batteryVoltage, uint16_t maxDuty, uint8_t motorPolesNum)
    : ENC_RES(encRes), BATTERY_VOLTAGE(batteryVoltage), MAX_DUTY(maxDuty), MOTOR_POLES_NUM(motorPolesNum), ENC_VAL_PER_ELEC_ANGLE((float)ENC_RES / (float)MOTOR_POLES_NUM) {
    enc = std::make_unique<Encoder>(ENC_RES);
    driver = std::make_unique<Driver>(BATTERY_VOLTAGE, MAX_DUTY);
    elecAngle = 0.0f;
    phase = 0.0f;
    turn = false;
}

Motor::~Motor() {}

void Motor::motorCalibrate() {
    int32_t encValSum = 0;
    for (int i = 0; i < CALIBRATION_SAMPLE_NUM; i < i++) {
        for (int j = 0; j < MOTOR_POLES_NUM; j++) {
            for (int k = 0; k < 6; k++) {
                driver->driveSquareWave(BATTERY_VOLTAGE * 0.1f, k * PI_3);
                HAL_Delay(5);
            }
        }
        HAL_Delay(50);
        encValSum += enc->getVal(0);
    }
    for (int i = 0; i < CALIBRATION_SAMPLE_NUM; i < i++) {
        for (int j = 0; j < MOTOR_POLES_NUM; j++) {
            for (int k = 5; k > -1; k--) {
                driver->driveSquareWave(BATTERY_VOLTAGE * 0.1f, k * PI_3);
                HAL_Delay(5);
            }
        }
        HAL_Delay(50);
        encValSum += enc->getVal(0);
    }
    enc->setOffset(encValSum / (CALIBRATION_SAMPLE_NUM * MOTOR_POLES_NUM * 2));
}

void Motor::init() {
    this->enc->init();
    this->driver->init();
    motorCalibrate();
}

void Motor::calcElecAngle() {
    elecAngle = fmodf((float)enc->getVal(1), ENC_VAL_PER_ELEC_ANGLE) * 2.0f * PI / ENC_VAL_PER_ELEC_ANGLE;
}

void Motor::release() {
    driver->freeWheel();
}

void Motor::invertTurn() {
    turn = !turn;
}

float Motor::getElecAngle() {
    calcElecAngle();
    return elecAngle;
}

void Motor::calcPhase(bool turn) {
    if (!turn) {
        phase = fmodf(fmodf(elecAngle - ADVANCED_ANGLE, TWO_PI) + TWO_PI, TWO_PI);
    } else {
        phase = fmodf(fmodf(elecAngle + ADVANCED_ANGLE, TWO_PI) + TWO_PI, TWO_PI);
    }
}

float Motor::getPhase() {
    calcPhase(turn);
    return phase;
}