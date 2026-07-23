//
// Created by divyansh on 7/17/26.
//
#include "BalancePID.h"

namespace BipedalV1 {
    BalancePID::BalancePID(float Kp_roll, float Ki_roll, float Kd_roll, float Kp_pitch, float Ki_pitch, float Kd_pitch,
                           unsigned int frequency) {
        m_pitch.kp = Kp_pitch;
        m_pitch.ki = Ki_pitch;
        m_pitch.kd = Kd_pitch;

        m_roll.kp = Kp_roll;
        m_roll.ki = Ki_roll;
        m_roll.kd = Kd_roll;
    }

    float BalancePID::getPitchPID(float pitch) {
        if (m_pitch.last_time == 0) {
            m_pitch.last_time = STM32F411::Clock::micros();
        }
        const float dt = m_pitch.last_time - STM32F411::Clock::micros();
        const float error_pitch = m_pitch.target - pitch;
        m_pitch.target += error_pitch * dt;
        const float derivative = (error_pitch - m_pitch.last_error) / dt;

        const float m_pid_output = m_pitch.kp * error_pitch + m_pitch.ki * m_pitch.integral + m_pitch.kd * derivative;


        m_pitch.last_error = error_pitch;
        m_pitch.last_time = STM32F411::Clock::micros();
        return m_pid_output;
    }

    float BalancePID::getRollPID(float roll) {
        if (m_roll.last_time == 0) {
            m_roll.last_time = STM32F411::Clock::micros();
        }
        const float dt = m_roll.last_time - STM32F411::Clock::micros();
        const float error_pitch = m_roll.target - roll;
        m_roll.target += error_pitch * dt;
        const float derivative = (error_pitch - m_roll.last_error) / dt;

        const float m_pid_output = m_roll.kp * error_pitch + m_roll.ki * m_roll.integral + m_roll.kd * derivative;


        m_roll.last_error = error_pitch;
        m_roll.last_time = STM32F411::Clock::micros();
        return m_pid_output;
    }
};
