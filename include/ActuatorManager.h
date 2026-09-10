//
// Created by divyansh on 7/15/26.
//

#ifndef BIPEDALV1_ACTUATORMANAGER_H
#define BIPEDALV1_ACTUATORMANAGER_H
#include "drivers/GPIO.h"
#include "drivers/PWM.h"

namespace Biped {
    class LockedAntiPhaseSpeed {
        float m_normalized_speed;

    public:
        constexpr explicit LockedAntiPhaseSpeed(float speed)
            : m_normalized_speed(speed < -1.0f ? -1.0f : (speed > 1.0f ? 1.0f : speed)) {}

        [[nodiscard]] constexpr float toDuty() const {
            return (m_normalized_speed + 1.0f) * 0.5f;
        }

        [[nodiscard]] constexpr float toInvertedDuty() const {
            return 1.0f - toDuty();
        }
    };
    class ActuatorManager {
        Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel2> m_left_wheel_pwm =
                Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel2>();
        Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel3> m_right_wheel_pwm =
                Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel3>();
        Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel1> m_thigh_left_pwm =
                       Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel1>();
        Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel4> m_thigh_right_pwm=
                Biped::PWM::PWM<Biped::PWM::Timer::TIMER5, Biped::PWM::TimerChannel::Channel4>();

        using phased_anti_lock_pwm_enable = Biped::Pins::A7;
        using upper_left_dir_pin = Biped::Pins::A5;
        using upper_right_dir_pin = Biped::Pins::A6;



        using m_pin_left_wheel_dir = Biped::Pins::A1;
        using m_pin_right_wheel_dir = Biped::Pins::A2;
        using m_pin_left_thigh_pwm = Biped::Pins::A0;
        using m_pin_right_thigh_pwm = Biped::Pins::A3;
    public:
        ActuatorManager() = default;
        ActuatorManager(ActuatorManager& other) = delete;
        ActuatorManager(ActuatorManager&& other) = delete;
        
        void initialize();

        void enableWheels();


        void rotateHip(float speed_left, float speed_right);

        void setLeftWheel(const LockedAntiPhaseSpeed speed);
        void setRightWheel(const LockedAntiPhaseSpeed speed);
        void move(const float speed_left, const float speed_right);

        void update();
    };
}
#endif //BIPEDALV1_ACTUATORMANAGER_H
