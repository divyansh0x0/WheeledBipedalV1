#include "ServoManager.h"

#include <numbers>

#include "BalancePID.h"
#include "Context.h"

namespace Biped {
    constexpr float MAX_ROLL_ANGLE = 45;
    static BalancePID balance_pid{0.1350f, 0.0f, 0.0040f, 0.1350f, 0.0f, 0.0040f, 0, 0};

    void ServoManager::setPWM(float left_wheel, float right_wheel) {
        const LockedAntiPhaseSpeed left_pwm{left_wheel};
        const LockedAntiPhaseSpeed right_pwm{right_wheel};

        const auto right_wheel_pwm = right_pwm.toDuty();
        m_left_wheel_pwm.setDutyCycle(left_pwm.toInvertedDuty());
        m_right_wheel_pwm.setDutyCycle(right_wheel_pwm);
    }

    void ServoManager::initialize(float wheel_radius, float hip_joint_radius, AS5600::AS5600State *wheel_left,
                                  AS5600::AS5600State *wheel_right, AS5600::AS5600State *hip_left,
                                  AS5600::AS5600State *hip_right) {
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

        // Enable timers to update generating the PWM signal internally
        m_left_wheel_pwm.enable();
        m_right_wheel_pwm.enable();
        m_thigh_left_pwm.enable();
        m_thigh_right_pwm.enable();

        // Route the fully configured timer signals to the GPIO pins
        m_pin_left_wheel_dir::enableAlternateFunction<Biped::Peripherals::TIMER5>();
        m_pin_right_wheel_dir::enableAlternateFunction<Biped::Peripherals::TIMER5>();
        m_pin_left_thigh_pwm::enableAlternateFunction<Biped::Peripherals::TIMER5>();
        m_pin_right_thigh_pwm::enableAlternateFunction<Biped::Peripherals::TIMER5>();

        this->servos.hip_left = hip_left;
        this->servos.hip_right = hip_right;
        this->servos.wheel_left = wheel_left;
        this->servos.wheel_right = wheel_right;
        this->servos.wheel_right = wheel_right;

        constexpr float Kp = 0.001;
        constexpr float Ki = 0.003;

        l_wheel_pid_config.kp = Kp;
        l_wheel_pid_config.ki = Ki;
        l_wheel_pid_config.integral_max = 1.0f/Ki;
        l_wheel_pid_config.output_max = 1.0f;

        r_wheel_pid_config.kp = Kp;
        r_wheel_pid_config.ki = Ki;
        r_wheel_pid_config.integral_max = 1.0f/Ki;
        r_wheel_pid_config.output_max = 1.0f;

        this->l_wheel_pi_controller.initialize(&l_wheel_pid_config, nullptr);
        this->r_wheel_pi_controller.initialize(&r_wheel_pid_config, nullptr);
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

    void ServoManager::setLeftWheelRPM(float rpm) {
        l_wheel_pid_config.target = rpm;
    }

    void ServoManager::setRightWheelRPM(float rpm) {
        r_wheel_pid_config.target = -rpm;
    }

    void ServoManager::setWheelRPM(const float rpm_left, const float rpm_right) {
        setLeftWheelRPM(rpm_left);
        setRightWheelRPM(rpm_right);
    }


    void ServoManager::update() {
        const float left_wheel_pid_output = l_wheel_pi_controller.getValue(servos.wheel_left->rpm, 0);
        const float right_wheel_pid_output = r_wheel_pi_controller.getValue(servos.wheel_right->rpm, 0);

        setPWM(left_wheel_pid_output, right_wheel_pid_output);
    }
}
