#ifndef BIPEDALV1_BALANCEPID_H
#define BIPEDALV1_BALANCEPID_H

#include "PIDController.h"

namespace Biped {
    struct WheelSpeed {
        float left;
        float right;
    };

    class BalancePID {
        PID::PIDVars &m_balance_vars;
        PID::PIDVars &m_speed_vars;
        PID::PIDController m_balance;
        PID::PIDController m_speed;

    public:
        BalancePID(PID::PIDVars &balance, PID::PIDVars &speed);

        void setTargetAngle(const float angle) { m_balance_vars.target = angle; }
        void setTargetSpeed(const float rpm) { m_speed_vars.target = rpm; }

        void reset();

        float getBalancePIDMotorSpeed(float pitch_angle, float gyro_y);
        float getTargetAngle(float wheel_rpm_left, float wheel_rpm_right);
    };
}
#endif //BIPEDALV1_BALANCEPID_H