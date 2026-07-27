//
// Created by divyansh on 7/17/26.
//
#include "BalancePID.h"

namespace BipedalV1 {
    BalancePID::BalancePID(float Kp_roll, float Ki_roll, float Kd_roll, float Kp_pitch, float Ki_pitch, float Kd_pitch) {
        m_pitch.kp = Kp_pitch;
        m_pitch.ki = Ki_pitch;
        m_pitch.kd = Kd_pitch;

        m_roll.kp = Kp_roll;
        m_roll.ki = Ki_roll;
        m_roll.kd = Kd_roll;
    }

    float BalancePID::getPitchPID(float pitch) {
        const uint32_t now = STM32F411::Clock::micros();
        if (m_pitch.last_time == 0) {
            m_pitch.last_time = now;
            m_pitch.last_error = m_pitch.target - pitch;
            return 0.0f;  // No valid dt on first call, skip
        }
        const float dt = static_cast<float>(now - m_pitch.last_time) / 1'000'000.0f;
        if (dt <= 0.0f) return 0.0f;  // Guard against zero/negative dt

        const float error = m_pitch.target - pitch;

        // Accumulate integral and clamp to prevent windup
        m_pitch.integral += error * dt;
        m_pitch.integral = clamp(m_pitch.integral, -m_pitch.integral_max, m_pitch.integral_max);

        const float derivative = (error - m_pitch.last_error) / dt;

        const float output = m_pitch.kp * error + m_pitch.ki * m_pitch.integral + m_pitch.kd * derivative;

        m_pitch.last_error = error;
        m_pitch.last_time = now;

        // Clamp output to [-1, 1] for LockedAntiPhaseSpeed
        return clamp(output, -m_pitch.output_max, m_pitch.output_max);
    }

    float BalancePID::getRollPID(float roll) {
        const uint32_t now = STM32F411::Clock::micros();
        if (m_roll.last_time == 0) {
            m_roll.last_time = now;
            m_roll.last_error = m_roll.target - roll;
            return 0.0f;  // No valid dt on first call, skip
        }
        const float dt = static_cast<float>(now - m_roll.last_time) / 1'000'000.0f;
        if (dt <= 0.0f) return 0.0f;  // Guard against zero/negative dt

        const float error = m_roll.target - roll;

        // Accumulate integral and clamp to prevent windup
        m_roll.integral += error * dt;
        m_roll.integral = clamp(m_roll.integral, -m_roll.integral_max, m_roll.integral_max);

        const float derivative = (error - m_roll.last_error) / dt;

        const float output = m_roll.kp * error + m_roll.ki * m_roll.integral + m_roll.kd * derivative;

        m_roll.last_error = error;
        m_roll.last_time = now;

        // Clamp output to [-1, 1] for LockedAntiPhaseSpeed
        return clamp(output, -m_roll.output_max, m_roll.output_max);
    }
};
