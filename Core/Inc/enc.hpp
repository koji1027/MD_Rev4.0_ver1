#ifndef ENC_HPP
#define ENC_HPP

#include "i2c.h"
#include "stm32g4xx_hal.h"

#define ENC_ADDR 0x36
#define ENC_REG_RAW_ANGLE 0x0C

class Encoder {
   public:
    Encoder(uint16_t resolution = 4096);
    ~Encoder();

    void init();
    void update();
    void setOffset(uint16_t offsetVal);
    uint16_t getVal(uint8_t mode = 0);

    const uint16_t RESOLUTION;

   private:
    uint16_t rawVal;
    uint16_t val;
    uint16_t offsetVal;
};
#endif  // ENC_HPP