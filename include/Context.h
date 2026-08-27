//
// Created by divyansh on 8/27/26.
//

#ifndef BIPEDALV1_CONTEXT_H
#define BIPEDALV1_CONTEXT_H
#include "drivers/MPU6050.h"
inline STM32F411::MPU6050::MPU6050<STM32F411::I2C2, STM32F411::MPU6050::GyroScale::_250,
    STM32F411::MPU6050::AccelScale::g2> mpu6050({
    .gx = 1.0082f, .gy = 7.8047f, .gz = 0.5791, .ax = -0.007324, .ay = -0.01074
});

namespace Biped::Context {
    float getRoll();

    float getGyroY();

    float getGyroX();

    float getPitch();

    [[noreturn]] void initialize();

    void update();;
}
#endif //BIPEDALV1_CONTEXT_H
