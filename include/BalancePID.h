//
// Created by divyansh on 7/17/26.
//

#ifndef BIPEDALV1_BALANCEPID_H
#define BIPEDALV1_BALANCEPID_H

#include "PIDController.h"

namespace Biped {
    struct WheelSpeed {
        float left;
        float right;
    };

    /**
     * Cascaded balance controller built from two PIDControllers:
     *
     *   outer loop (speed):   avg wheel RPM error -> pitch angle offset (deg)
     *   inner loop (balance): pitch angle error   -> motor command [-output_max, output_max]
     *
     * The PIDVars are owned by the caller (gains, limits, targets, and the controller
     * state all live in them), so they can still be tweaked live, e.g. from CubeMonitor.
     */
    class BalancePID {
        PID::PIDVars &m_balance_vars;
        PID::PIDVars &m_speed_vars;
        PID::PIDController m_balance;
        PID::PIDController m_speed;

    public:
        BalancePID(PID::PIDVars &balance, PID::PIDVars &speed);

        /// Target pitch angle for the inner loop (deg).
        void setTargetAngle(const float angle) { m_balance_vars.target = angle; }

        /// Target average wheel speed for the outer loop (RPM).
        void setTargetSpeed(const float rpm) { m_speed_vars.target = rpm; }

        void reset();

        /// Inner loop. Returns the motor command for the given pitch and pitch rate (gyro_y).
        float getBalancePIDMotorSpeed(float pitch_angle, float gyro_y);

        /// Outer loop. Returns the pitch angle offset (deg) to add to the base/CoM angle.
        float getTargetAngle(float wheel_rpm_left, float wheel_rpm_right);
    };
}
#endif //BIPEDALV1_BALANCEPID_H