#include "motor.hpp"

Motor::Motor(uint16_t encRes, float batteryVoltage, uint16_t maxDuty, uint8_t motorPolesNum, uint16_t currentAdcRes, float currentAdcRefVoltage, uint8_t currentAmpGain, float currentShuntResistance)
    : ENC_RES(encRes), BATTERY_VOLTAGE(batteryVoltage), MAX_DUTY(maxDuty), MOTOR_POLES_NUM(motorPolesNum), ENC_VAL_PER_ELEC_ANGLE((float)ENC_RES / (float)MOTOR_POLES_NUM) {
    enc = std::make_unique<Encoder>(ENC_RES);
    driver = std::make_unique<Driver>(BATTERY_VOLTAGE, MAX_DUTY);
    currentSensor = std::make_unique<CurrentSensor>(currentAdcRes, currentAdcRefVoltage, currentAmpGain, currentShuntResistance);
}

Motor::~Motor() {}

void Motor::timerStart() {
    HAL_TIM_Base_Start_IT(&htim15);
    HAL_TIM_Base_Start_IT(&htim16);
    HAL_TIM_Base_Start_IT(&htim17);
}

void Motor::motorCalibrate() {
    enc->setOffset(0);
    uint32_t encRawValSum = 0;
    uint16_t encRawValArray[6 * MOTOR_POLES_NUM * MOTOR_CALIBRATION_SAMPLE_NUM];
    for (uint8_t i = 0; i < 6 * MOTOR_POLES_NUM; i++) {
        driver->driveSquareWave(2.0f, i);
        HAL_Delay(20);
    }
    for (uint8_t i = 0; i < MOTOR_CALIBRATION_SAMPLE_NUM; i++) {
        for (uint8_t j = 0; j < MOTOR_POLES_NUM; j++) {
            for (uint8_t k = 0; k < 6; k++) {
                driver->driveSquareWave(2.0f, k);
                HAL_Delay(20);
                encRawValArray[i * 6 * MOTOR_POLES_NUM + j * 6 + k] = enc->getVal(0);
                if ((i % 6 == 3 || i % 6 == 4) && j == MOTOR_POLES_NUM / 2) {
                    encRawValSum += encRawValArray[i * 6 * MOTOR_POLES_NUM + j * 6 + k];
                }
                HAL_Delay(20);
            }
        }
    }
    uint16_t encRawValOffset = encRawValSum / (2 * MOTOR_CALIBRATION_SAMPLE_NUM);
    // for (uint16_t i = 0; i < 6 * MOTOR_POLES_NUM * MOTOR_CALIBRATION_SAMPLE_NUM; i++) {
    //     char msg[1000];
    //     uint16_t encRawVal = encRawValArray[i];
    //     int32_t encValOffseted = (int32_t)encRawVal - (int32_t)encRawValOffset;
    //     if (encValOffseted < 0) {
    //         encValOffseted += ENC_RES;
    //     }
    //     uint8_t len = sprintf(msg, "%d,%d,%d\n", i, encRawVal, (uint16_t)encValOffseted);
    //     HAL_UART_Transmit_DMA(&huart2, (uint8_t *)msg, len);
    //     HAL_Delay(10);
    // }
    driver->freeWheel();
    enc->setOffset(encRawValOffset);
    HAL_Delay(200);
}

void Motor::init() {
    this->enc->init();
    this->driver->init();
    motorCalibrate();
    this->driver->freeWheel();
    HAL_Delay(200);
    this->currentSensor->init();
}

void Motor::calcElecAngle() {
    elecAngle = fmodf((float)enc->getVal(1), ENC_VAL_PER_ELEC_ANGLE) * 2.0f * PI / ENC_VAL_PER_ELEC_ANGLE;
}

void Motor::release() {
    driver->freeWheel();
}

void Motor::setTurn(bool turn) {
    this->turn = turn;
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
void Motor::calcRpm(uint64_t time_us) {
    static uint64_t prevTime = 0;
    float interval = (float)(time_us - prevTime);
    if (interval < 0) {
        return;
    }
    static float rpmBuf[RPM_MOVING_AVERAGE_SAMPLE_NUM];
    static uint8_t rpmBufIndex = 0;
    static float prevElecAngle = 0.0f;
    float diff = elecAngle - prevElecAngle;
    if (diff > PI) {
        diff -= TWO_PI;
    } else if (diff < -PI) {
        diff += TWO_PI;
    }
    if (abs(diff) < PI_3) {
        return;
    }
    prevTime = time_us;
    rpmBuf[rpmBufIndex] = diff / (interval / 1000000.0f) * 60.0f / TWO_PI;
    prevElecAngle = elecAngle;
    rpmBufIndex = (rpmBufIndex + 1) % RPM_MOVING_AVERAGE_SAMPLE_NUM;

    rpm = 0.0f;
    for (uint8_t i = 0; i < RPM_MOVING_AVERAGE_SAMPLE_NUM; i++) {
        rpm += rpmBuf[i];
    }
    rpm /= (float)RPM_MOVING_AVERAGE_SAMPLE_NUM;
}

float Motor::getRpm() {
    return rpm;
}