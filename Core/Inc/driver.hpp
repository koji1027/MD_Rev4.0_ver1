#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "stm32g4xx.h"

class Driver {
   public:
    Driver();
    ~Driver();

    void init();
    void driveSquareWave(float voltage, float phase);
    void driveSinWave(float voltage, float phase);

   private:
    void drive(float voltage[3]);
};

#endif  // DRIVER_HPP