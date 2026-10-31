#include "BalancePID.h"

namespace Biped {
    BalancePID::BalancePID(PID::PIDVars &balance, PID::PIDVars &speed)
        : m_balance_vars{balance}, m_speed_vars{speed} {
        m_balance.initialize(&m_balance_vars, nullptr);
        m_speed.initialize(&m_speed_vars, nullptr);
    }

    void BalancePID::reset() {
        m_balance.reset();
        m_speed.reset();
    }

    float BalancePID::getBalancePIDMotorSpeed(const float pitch_angle, const float gyro_y) {
        return m_balance.getValue(pitch_angle, gyro_y);
    }

    float BalancePID::getTargetAngle(const float wheel_rpm_left, const float wheel_rpm_right) {
        const float avg_rpm = 0.5f * (wheel_rpm_left + wheel_rpm_right);
        return m_speed.getValue(avg_rpm);
    }
}
