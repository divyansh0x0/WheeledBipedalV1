//
// Created by divyansh on 7/15/26.
//

#ifndef BIPEDALV1_ACTUATORMANAGER_H
#define BIPEDALV1_ACTUATORMANAGER_H
#include "drivers/GPIO.h"
#include "drivers/PWM.h"

namespace BipedalV1 {
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
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel2> m_left_wheel_pwm =
                STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel2>();
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel3> m_right_wheel_pwm =
                STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel3>();
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel1> m_thigh_left_pwm =
                       STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel1>();
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel4> m_thigh_right_pwm=
                STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER5, STM32F411::PWM::TimerChannel::Channel4>();

        using phased_anti_lock_pwm_enable = STM32F411::Pins::A7;
        using upper_left_dir_pin = STM32F411::Pins::A5;
        using upper_right_dir_pin = STM32F411::Pins::A4;



        using m_left_wheel_dir = STM32F411::Pins::A1;
        using m_right_wheel_dir = STM32F411::Pins::A2;
        using m_left_thigh_pwm = STM32F411::Pins::A0;
        using m_right_thigh_pwm = STM32F411::Pins::A3;
    public:
        ActuatorManager() = default;
        ActuatorManager(ActuatorManager& other) = delete;
        ActuatorManager(ActuatorManager&& other) = delete;
        void  initialize() {
            STM32F411::MemoryMap::RCC1->enablePeripheral(STM32F411::MemoryMap::AHB1Peripheral::GPIOA);
            STM32F411::MemoryMap::RCC1->enablePeripheral(STM32F411::MemoryMap::APB1Peripheral::TIMER5);


            phased_anti_lock_pwm_enable::enableOutputMode();
            phased_anti_lock_pwm_enable::set(STM32F411::GPIOStatus::LOW);
            upper_left_dir_pin::enableOutputMode();
            upper_left_dir_pin::set(STM32F411::GPIOStatus::LOW);
            upper_right_dir_pin::enableOutputMode();
            upper_right_dir_pin::set(STM32F411::GPIOStatus::LOW);


            m_left_wheel_dir::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();
            m_right_wheel_dir::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();
            m_left_thigh_pwm::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();
            m_right_thigh_pwm::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();

            // Set frequency FIRST so ARR is valid before channel outputs are enabled
            m_left_wheel_pwm.setFrequency(32000);
            m_right_wheel_pwm.setFrequency(32000);
            m_thigh_left_pwm.setFrequency(32000);
            m_thigh_right_pwm.setFrequency(32000);

            // Initialize phased anti-lock pwm channels to 50% duty (stopped in locked anti-phase)
            m_left_wheel_pwm.setDutyCycle(0.5f);
            m_right_wheel_pwm.setDutyCycle(0.5f);

            // Set duty of sign magnitude to zero
            m_thigh_left_pwm.setDutyCycle(0);
            m_thigh_right_pwm.setDutyCycle(0);

            m_left_wheel_pwm.enable();
            m_right_wheel_pwm.enable();

            m_thigh_left_pwm.enable();
            m_thigh_right_pwm.enable();

            phased_anti_lock_pwm_enable::set(STM32F411::HIGH);
        }

        void setLeftWheel(const LockedAntiPhaseSpeed speed) {
            m_left_wheel_pwm.setDutyCycle(speed.toDuty());
        }
        void setRightWheel(const LockedAntiPhaseSpeed speed) {
            m_right_wheel_pwm.setDutyCycle(speed.toDuty());
        }

        void move(const float speed_left,const float speed_right) {
            const LockedAntiPhaseSpeed targetSpeedLeft(speed_left);
            const LockedAntiPhaseSpeed targetSpeedRight(speed_right);
            setLeftWheel(targetSpeedLeft);
            setRightWheel(targetSpeedRight);
        }
    };
}
#endif //BIPEDALV1_ACTUATORMANAGER_H
