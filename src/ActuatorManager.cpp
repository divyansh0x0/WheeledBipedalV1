#include "ActuatorManager.h"

namespace BipedalV1 {

    void ActuatorManager::initialize() {
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

    void ActuatorManager::setLeftWheel(const LockedAntiPhaseSpeed speed) {
        m_left_wheel_pwm.setDutyCycle(speed.toDuty());
    }

    void ActuatorManager::setRightWheel(const LockedAntiPhaseSpeed speed) {
        m_right_wheel_pwm.setDutyCycle(speed.toDuty());
    }

    void ActuatorManager::move(const float speed_left, const float speed_right) {
        const LockedAntiPhaseSpeed targetSpeedLeft(speed_left);
        const LockedAntiPhaseSpeed targetSpeedRight(speed_right);
        setLeftWheel(targetSpeedLeft);
        setRightWheel(targetSpeedRight);
    }

}
