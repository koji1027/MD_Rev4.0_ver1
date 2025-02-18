#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "stm32g4xx.h"

class Driver {
   public:
    Driver(uint16_t max_voltage, uint16_t max_duty);
    ~Driver();

    void init();
    void driveSquareWave(float voltage, float phase);
    void driveSinWave(float voltage, float phase);
    void freeWheel();

   private:
    void drive(float voltage[3]);

    const uint16_t MAX_VOLTAGE;
    const uint16_t MAX_DUTY;
};

#endif  // DRIVER_HPP