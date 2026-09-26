//
// Created by divyansh on 7/15/26.
//

#ifndef BIPEDALV1_ACTUATORMANAGER_H
#define BIPEDALV1_ACTUATORMANAGER_H
#include "EncoderI2CMux.h"
#include "PIDController.h"
#include "drivers/GPIO.h"
#include "drivers/PWM.h"

namespace Biped {
    struct ServoState {
        float radius = 0;
        float target_angle = 0;
        MagneticEncoder::EncoderI2CMux* encoder_state = nullptr;
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
        PID::PIDVars l_wheel_pid_config_position{};
        PID::PIDController l_wheel_pi_controller{};
        PID::PIDController r_wheel_pi_controller{};

        F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel2> m_right_wheel_pwm =
                F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel2>();
        F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel3> m_left_wheel_pwm =
                F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel3>();
        F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel1> m_thigh_left_pwm =
                F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel1>();
        F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel4> m_thigh_right_pwm =
                F411::PWM::PWM<F411::PWM::Timer::TIMER5, F411::PWM::TimerChannel::Channel4>();

        using phased_anti_lock_pwm_enable = F411::Pins::A7;
        using upper_left_dir_pin = F411::Pins::A5;
        using upper_right_dir_pin = F411::Pins::A6;


        using m_pin_left_wheel_dir = F411::Pins::A1;
        using m_pin_right_wheel_dir = F411::Pins::A2;
        using m_pin_left_thigh_pwm = F411::Pins::A0;
        using m_pin_right_thigh_pwm = F411::Pins::A3;

        struct ServoStates {
            MagneticEncoder::Encoder* wheel_left{};
            MagneticEncoder::Encoder* wheel_right{};
            MagneticEncoder::Encoder* hip_left{};
            MagneticEncoder::Encoder* hip_right{};
        } servos = {};
        void setPWM(float left_wheel, float right_wheel);
    public:
        ServoManager() = default;
        ServoManager(ServoManager& other) = delete;
        ServoManager(ServoManager&& other) = delete;
        
        void initialize(float wheel_radius, float hip_joint_radius,MagneticEncoder::Encoder* wheel_left, MagneticEncoder::Encoder* wheel_right, MagneticEncoder::Encoder* hip_left, MagneticEncoder::Encoder* hip_right);

        void enableWheels();


        void rotateHip(float speed_left, float speed_right);

        void setLeftWheelRPM(float speed);
        void setRightWheelRPM(float speed);
        void setWheelRPM(const float speed_left, const float speed_right);

        void update();

        volatile float getRightWheelRPM(){return servos.wheel_right->rpm;}

        volatile float getLeftWheelRPM() {return servos.wheel_left->rpm;}
    };
}
#endif //BIPEDALV1_ACTUATORMANAGER_H
