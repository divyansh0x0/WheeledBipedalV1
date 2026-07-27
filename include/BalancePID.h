//
// Created by divyansh on 7/17/26.
//

#ifndef BIPEDALV1_BALANCEPID_H
#define BIPEDALV1_BALANCEPID_H
#include "drivers/MPU6050.h"

namespace BipedalV1 {
    struct WheelSpeed {
        float left;
        float right;
    };
    struct PIDVars {
        float kp = 0;
        float ki = 0;
        float kd = 0;
        float integral = 0;
        float integral_max = 0.5f;  // Anti-windup clamp
        float output_max = 1.0f;    // Clamp to [-1, 1] for LockedAntiPhaseSpeed
        float target = 0;
        float last_error = 0;
        unsigned int last_time = 0;
    };
    class BalancePID {
        WheelSpeed m_wheel_speed{};
        PIDVars m_roll{};
        PIDVars m_pitch{};

        static float clamp(float value, float min_val, float max_val) {
            return value < min_val ? min_val : (value > max_val ? max_val : value);
        }
    public:
        BalancePID(float Kp_roll, float Ki_roll, float Kd_roll, float Kp_pitch, float Ki_pitch, float Kd_pitch);


        float getPitchPID(float pitch);
        float getRollPID(float roll);
    };
}
#endif //BIPEDALV1_BALANCEPID_H
