//
// Created by divyansh on 9/12/26.
//

#include "PIDController.h"


#include "drivers/Clock.h"

template<typename  T>
static T clamp(T value, T min, T max) {
    return value <  min ? min :  value > max ? max : value;
}
namespace Biped::PID {
    void PIDController::initialize(PIDVars *pid_config, Limiter *limiter) {
        m_pid_config = pid_config;
        m_limiter = limiter;
    }

    float PIDController::getValue(float control_value, float control_value_rate_change) const {
        PIDVars pid = *m_pid_config;

        const unsigned int curr_time = Clock::micros();
        const unsigned int dt = curr_time - pid.last_time_us;
        pid.integral = static_cast<float>(dt) * (pid.target - control_value);
        pid.last_time_us = curr_time;

        pid.integral = clamp(pid.integral, -pid.integral_max, pid.integral_max);

        const float P = pid.kp * (pid.target - control_value);
        const float D = pid.kd * control_value_rate_change;
        const float I = pid.ki * (pid.integral);

        return P + D + I;
    }
} // Biped::PID
