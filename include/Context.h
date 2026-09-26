//
// Created by divyansh on 8/27/26.
//

#ifndef BIPEDALV1_CONTEXT_H
#define BIPEDALV1_CONTEXT_H
#include "EncoderI2CMux.h"

namespace Biped {
    class ServoManager;
}

namespace Biped::Context {
    float getRoll();

    float getGyroY();

    float getGyroX();

    MagneticEncoder::EncoderI2CMux *getEncoderI2CMux();

    float getPitch();

    float getCurrentHipLeft();

    float getCurrentHipRight();

    float getVoltageHipLeft();

    float getVoltageHipRight();

    ServoManager *getServoManager();

    void initialize();

    void update();

    float getBatteryPercentage();

    float getBatteryVoltage();
}
#endif //BIPEDALV1_CONTEXT_H
