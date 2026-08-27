#include "ActuatorManager.h"

#include <numbers>

#include "BalancePID.h"
#include "Context.h"

namespace Biped {
    constexpr float MAX_ROLL_ANGLE = 45;
    static BalancePID balance_pid{0.1350f, 0.0f, 0.0040f, 0.1350f, 0.0f, 0.0040f, 0, 0};

    static float doPID() {
        float pid_output = 0;
        const float roll = Context::getPitch();
        const float gyro_x = Context::getGyroX();
        if (roll > MAX_ROLL_ANGLE || roll < -MAX_ROLL_ANGLE) {
            pid_output = 0.0f;
            balance_pid.reset();
        } else {
            pid_output = balance_pid.getRollPID(roll, gyro_x);
        }
        return pid_output;
    }

    void ActuatorManager::initialize() {
        STM32F411::MemoryMap::RCC1->enablePeripheral(STM32F411::MemoryMap::AHB1Peripheral::GPIOA);
        STM32F411::MemoryMap::RCC1->enablePeripheral(STM32F411::MemoryMap::APB1Peripheral::TIMER5);

        phased_anti_lock_pwm_enable::enableOutputMode();
        phased_anti_lock_pwm_enable::set(STM32F411::GPIOStatus::LOW);
        upper_left_dir_pin::enableOutputMode();
        upper_left_dir_pin::set(STM32F411::GPIOStatus::LOW);
        upper_right_dir_pin::enableOutputMode();
        upper_right_dir_pin::set(STM32F411::GPIOStatus::LOW);

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

        // Enable timers to start generating the PWM signal internally
        m_left_wheel_pwm.enable();
        m_right_wheel_pwm.enable();
        m_thigh_left_pwm.enable();
        m_thigh_right_pwm.enable();

        // Route the fully configured timer signals to the GPIO pins
        m_left_wheel_dir::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();
        m_right_wheel_dir::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();
        m_left_thigh_pwm::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();
        m_right_thigh_pwm::enableAlternateFunction<STM32F411::Peripherals::TIMER5>();
    }

    void ActuatorManager::enableWheels() {
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

    void ActuatorManager::update() {
        float pid_output = doPID();
        this->move(pid_output, pid_output);
    }
}
