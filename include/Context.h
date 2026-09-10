//
// Created by divyansh on 8/27/26.
//

#ifndef BIPEDALV1_CONTEXT_H
#define BIPEDALV1_CONTEXT_H
#include "AS5600MUX.h"

namespace Biped::Context {
    float getRoll();

    float getGyroY();

    float getGyroX();

    AS5600::AS5600MUX* getAS5600MUX();
    float getPitch();
    float getCurrentHipLeft();
    float getCurrentHipRight();
    float getVoltageHipLeft();
    float getVoltageHipRight();

    void initialize();

    void update();

    float getBatteryPercentage();
    float getBatteryVoltage();
}
#endif //BIPEDALV1_CONTEXT_H
