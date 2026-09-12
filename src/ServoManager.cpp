#include "ServoManager.h"

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

    void ServoManager::initialize(float wheel_radius, float hip_joint_radius, AS5600::AS5600State* wheel_left, AS5600::AS5600State* wheel_right, AS5600::AS5600State* hip_left, AS5600::AS5600State* hip_right) {
        Biped::MemoryMap::RCC1->enablePeripheral(Biped::MemoryMap::AHB1Peripheral::GPIOA);
        Biped::MemoryMap::RCC1->enablePeripheral(Biped::MemoryMap::APB1Peripheral::TIMER5);

        phased_anti_lock_pwm_enable::enableOutputMode();
        phased_anti_lock_pwm_enable::set(Biped::GPIOStatus::LOW);
        upper_left_dir_pin::enableOutputMode();
        upper_left_dir_pin::set(Biped::GPIOStatus::LOW);
        upper_right_dir_pin::enableOutputMode();
        upper_right_dir_pin::set(Biped::GPIOStatus::LOW);

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
        m_pin_left_wheel_dir::enableAlternateFunction<Biped::Peripherals::TIMER5>();
        m_pin_right_wheel_dir::enableAlternateFunction<Biped::Peripherals::TIMER5>();
        m_pin_left_thigh_pwm::enableAlternateFunction<Biped::Peripherals::TIMER5>();
        m_pin_right_thigh_pwm::enableAlternateFunction<Biped::Peripherals::TIMER5>();

        this->servos.hip_left.encoder_state = hip_left;
        this->servos.hip_left.radius = hip_joint_radius;
        this->servos.hip_right.encoder_state = hip_right;
        this->servos.hip_right.radius = hip_joint_radius;
        this->servos.wheel_left.encoder_state = wheel_left;
        this->servos.wheel_left.radius = wheel_radius;
        this->servos.wheel_right.encoder_state = wheel_right;
        this->servos.wheel_right.radius = wheel_radius;
    }

    void ServoManager::enableWheels() {
        phased_anti_lock_pwm_enable::set(Biped::HIGH);
    }

    void ServoManager::rotateHip(const float speed_left, const float speed_right) {
        if (speed_left < 0) {
            upper_left_dir_pin::set(Biped::LOW);
        } else {
            upper_left_dir_pin::set(Biped::HIGH);
        }
        if (speed_right < 0) {
            upper_right_dir_pin::set(Biped::LOW);
        } else {
            upper_right_dir_pin::set(Biped::HIGH);
        }

        m_thigh_left_pwm.setDutyCycle(speed_left < 0 ? -speed_left : speed_left);
        m_thigh_right_pwm.setDutyCycle(speed_right < 0 ? -speed_right : speed_right);
    }

    void ServoManager::setLeftWheel(const LockedAntiPhaseSpeed speed) {
        m_left_wheel_pwm.setDutyCycle(speed.toDuty());
    }

    void ServoManager::setRightWheel(const LockedAntiPhaseSpeed speed) {
        m_right_wheel_pwm.setDutyCycle(speed.toDuty());
    }

    void ServoManager::set_wheel_speed(const float speed_left, const float speed_right) {
        const LockedAntiPhaseSpeed targetSpeedLeft(speed_left);
        const LockedAntiPhaseSpeed targetSpeedRight(speed_right);
        setLeftWheel(targetSpeedLeft);
        setRightWheel(targetSpeedRight);
    }

    void ServoManager::update() {
        float pid_output = doPID();
        this->set_wheel_speed(pid_output, pid_output);
    }
}
