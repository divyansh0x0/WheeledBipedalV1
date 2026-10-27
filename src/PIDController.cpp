//
// Created by divyansh on 9/12/26.
//

#include "PIDController.h"

#include "drivers/Clock.h"

template<typename T>
static T clamp(T value, T min, T max) {
    return value < min ? min : value > max ? max : value;
}

namespace Biped::PID {
    void PIDController::initialize(PIDVars *pid_config, Limiter *limiter) {
        m_pid_config = pid_config;
        m_limiter = limiter;
    }

    void PIDController::reset() {
        if (m_pid_config == nullptr) return;
        m_pid_config->integral = 0.0f;
        m_pid_config->last_error = 0.0f;
        m_pid_config->last_time_us = 0;
    }

    float PIDController::getValue(float control_value, float control_value_rate_change) {
        PIDVars* pid = m_pid_config;
        if (pid == nullptr) return 0.0f;

        const unsigned int curr_time = F411::Clock::micros();

        // First run initialization
        if (pid->last_time_us == 0) {
            pid->last_time_us = curr_time;
            pid->last_error = pid->target - control_value;
            return 0.0f;
        }

        const unsigned int dt = curr_time - pid->last_time_us;
        if (dt == 0) {
            // Guard against multiple calls in the same microsecond
            return 0.0f;
        }

        const float dt_sec = static_cast<float>(dt) * 1e-6f; // Convert microseconds to seconds

        // Prevent massive spikes after a pause or long gap
        if (dt_sec > 1.0f) {
            pid->last_time_us = curr_time;
            pid->last_error = pid->target - control_value;
            return 0.0f;
        }

        pid->last_time_us = curr_time;
        const float error = pid->target - control_value;

        // Proportional term
        const float P = pid->kp * error;

        // Integral term with output anti-windup clamping
        float I = 0.0f;
        if (pid->ki > 0.0f) {
            pid->integral += error * dt_sec;
            I = pid->ki * pid->integral;
            const float clamped_I = clamp(I, -pid->integral_max, pid->integral_max);
            if (I != clamped_I) {
                I = clamped_I;
                // Back-calculate integral accumulation to prevent windup
                pid->integral = I / pid->ki;
            }
        } else {
            pid->integral = 0.0f;
            I = 0.0f;
        }

        // Derivative term:
        // When rate change of measurement is provided, derivative-on-measurement damping opposes it (-rate).
        // If not provided (0.0f), fall back to calculating derivative of error over time.
        float derivative = 0.0f;
        if (control_value_rate_change != 0.0f) {
            derivative = -control_value_rate_change;
        } else if (dt_sec > 0.0f) {
            derivative = (error - pid->last_error) / dt_sec;
        }
        const float D = pid->kd * derivative;

        pid->last_error = error;

        // Total output clamped to configured maximum limit
        const float output = P + I + D;
        return clamp(output, -pid->output_max, pid->output_max);
    }
} // Biped::PID
