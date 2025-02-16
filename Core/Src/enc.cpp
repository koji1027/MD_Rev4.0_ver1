#include "enc.hpp"

Encoder::Encoder(uint16_t resolution) : RESOLUTION(resolution) {
    rawVal = 0;
    val = 0;
    offsetVal = 0;
}

Encoder::~Encoder() {}

void Encoder::init() {
}

void Encoder::update() {
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c2, ENC_ADDR << 1, ENC_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, data, 2, 1);
    rawVal = (data[0] << 8) | data[1];
    val = ((rawVal + offsetVal) % RESOLUTION + RESOLUTION) % RESOLUTION;
}

void Encoder::setOffset(uint16_t offsetVal) {
    this->offsetVal = offsetVal;
}

uint16_t Encoder::getVal(uint8_t mode) {
    if (mode == 0) {
        return rawVal;
    } else if (mode == 1) {
        return val;
    } else if (mode == 2) {
        return offsetVal;
    } else {
        return 0;
    }
}