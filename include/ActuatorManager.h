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
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER2, STM32F411::PWM::TimerChannel::Channel1> m_pwm_lower_left =
                STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER2, STM32F411::PWM::TimerChannel::Channel1>();
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER2, STM32F411::PWM::TimerChannel::Channel2> m_pwm_lower_right =
                STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER2, STM32F411::PWM::TimerChannel::Channel2>();

    public:
        ActuatorManager() = default;
        ActuatorManager(ActuatorManager& other) = delete;
        ActuatorManager(ActuatorManager&& other) = delete;
        void  initialize() {
            STM32F411::MemoryMap::RCC1->enablePeripheral(STM32F411::MemoryMap::AHB1Peripheral::GPIOA);

            using m_left_wheel = STM32F411::Pins::A1;
            using m_right_wheel = STM32F411::Pins::A2;
            using phased_anti_lock_pwm_enable = STM32F411::Pins::A4;


            phased_anti_lock_pwm_enable::enableOutputMode();
            phased_anti_lock_pwm_enable::set(STM32F411::GPIOStatus::LOW);

            m_left_wheel::enableAlternateFunction<STM32F411::Peripherals::TIMER2>();
            m_right_wheel::enableAlternateFunction<STM32F411::Peripherals::TIMER2>();


            m_pwm_lower_left.enable();
            m_pwm_lower_right.enable();

            m_pwm_lower_left.setFrequency(32000);
            m_pwm_lower_right.setFrequency(32000);
            phased_anti_lock_pwm_enable::set(STM32F411::GPIOStatus::HIGH);
        }

        void setLeftWheel(const LockedAntiPhaseSpeed speed) {
            m_pwm_lower_left.setDutyCycle(speed.toDuty());
        }
        void setRightWheel(const LockedAntiPhaseSpeed speed) {
            m_pwm_lower_right.setDutyCycle(speed.toInvertedDuty());
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
