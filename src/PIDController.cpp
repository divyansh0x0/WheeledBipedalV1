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
        PIDVars* pid = m_pid_config;

        const unsigned int curr_time = Clock::micros();
        const unsigned int dt = curr_time - pid->last_time_us;
        const float dt_sec = static_cast<float>(dt) * 1e-6f; // Convert microseconds to seconds
        
        // Prevent massive spikes on the very first loop or after a long pause
        if (dt_sec > 1.0f || pid->last_time_us == 0) {
            pid->last_time_us = curr_time;
            pid->last_error = pid->target - control_value;
            return 0.0f; 
        }
        
        pid->last_time_us = curr_time;
        const float error = pid->target - control_value;

        // I-Term: Accumulate integral scaled perfectly by seconds
        pid->integral += error * dt_sec;
        pid->integral = clamp(pid->integral, -pid->integral_max, pid->integral_max);

        // P, I, D calculations
        const float P = pid->kp * error;
        const float I = pid->ki * pid->integral;
        
        // D-term uses provided rate. (For Derivative-on-Measurement stability, Kd is typically negative)
        const float D = pid->kd * control_value_rate_change; 
        
        float output = P + I + D;
        
        // Clamp total output safely
        output = clamp(output, -pid->output_max, pid->output_max);
        
        pid->last_error = error;
        return output;
    }
} // Biped::PID
