//
// Created by divyansh on 9/12/26.
//

#ifndef BIPEDALV1_CONTROLLER_H
#define BIPEDALV1_CONTROLLER_H
#include <numbers>

namespace Biped::PID {
    class Limiter {
    };

    struct PIDVars {
        float kp = 0;
        float ki = 0;
        float kd = 0;
        float integral = 0;
        float integral_max = 0.5f;
        float output_max = 1.0f;
        float target = 0.0f;
        float last_error = 0;
        unsigned int last_time_us = 0;
    };

    class PIDController {
        PIDVars *m_pid_config;
        Limiter *m_limiter;

    public:
        void initialize(PIDVars *pid_config, Limiter *limiter);

        [[nodiscard]] float getValue(float control_value, float control_value_rate_change) const;
    };
}
#endif //BIPEDALV1_CONTROLLER_H
