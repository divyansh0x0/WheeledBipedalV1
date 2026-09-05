//
// Created by divyansh on 8/27/26.
//

#ifndef BIPEDALV1_CONTEXT_H
#define BIPEDALV1_CONTEXT_H
namespace Biped::Context {
    float getRoll();

    float getGyroY();

    float getGyroX();

    float getPitch();
    float getCurrentHipLeft();
    float getCurrentHipRight();
    float getVoltageHipLeft();
    float getVoltageHipRight();

    void initialize();

    void update();

    float getBatteryPercentage();
}
#endif //BIPEDALV1_CONTEXT_H
