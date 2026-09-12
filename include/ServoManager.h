//
// Created by divyansh on 7/15/26.
//

#ifndef BIPEDALV1_ACTUATORMANAGER_H
#define BIPEDALV1_ACTUATORMANAGER_H
#include "AS5600MUX.h"
#include "PIDController.h"
#include "drivers/GPIO.h"
#include "drivers/PWM.h"

namespace Biped {
    struct ServoState {
        float radius = 0;
        float target_angle = 0;
        AS5600::AS5600State* encoder_state = nullptr;
        float dt = 0;
        float duty_cycle = 0;
    };
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
    class ServoManager {

        PID::PIDVars l_wheel_pid_config{};
        PID::PIDVars r_wheel_pid_config{};
        PID::PIDController l_wheel_pi_controller{};
        PID::PIDController r_wheel_pi_controller{};

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

        struct Servos {
            ServoState wheel_left{};
            ServoState wheel_right{};
            ServoState hip_left{};
            ServoState hip_right{};
        } servos = {};
    public:
        ServoManager() = default;
        ServoManager(ServoManager& other) = delete;
        ServoManager(ServoManager&& other) = delete;
        
        void initialize(float wheel_radius, float hip_joint_radius,AS5600::AS5600State* wheel_left, AS5600::AS5600State* wheel_right, AS5600::AS5600State* hip_left, AS5600::AS5600State* hip_right);

        void enableWheels();


        void rotateHip(float speed_left, float speed_right);

        void setLeftWheelRPM(float speed);
        void setRightWheelRPM(float speed);
        void setWheelRPM(const float speed_left, const float speed_right);

        void update();
    };
}
#endif //BIPEDALV1_ACTUATORMANAGER_H
