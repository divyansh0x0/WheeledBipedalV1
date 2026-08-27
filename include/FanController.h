//
// Created by divyansh on 8/26/26.
//


#ifndef BIPEDALV1_FANCONTROLLER_H
#define BIPEDALV1_FANCONTROLLER_H
#include "drivers/GPIO.h"
#include "drivers/PWM.h"

namespace BipedalV1 {
    class FanController {
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER3, STM32F411::PWM::TimerChannel::Channel4> m_pwm =
                STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER3, STM32F411::PWM::TimerChannel::Channel4>();

    public:
        FanController() = default;

        FanController(FanController &&buzzer) = delete;

        FanController(FanController &buzzer) = delete;

        void initialize() {
            // STM32F411::Pins::B1::enableAlternateFunction<STM32F411::Peripherals::TIMER3>();
            // m_pwm.enable();
            // m_pwm.setDutyCycle(0);
            STM32F411::Pins::B1::enableOutputMode();
            STM32F411::Pins::B1::set(STM32F411::LOW);
        }

        void setDutyCycle(const float duty_cycle) {
            // m_pwm.setDutyCycle(m_duty_cycle);
            STM32F411::Pins::B1::set(STM32F411::HIGH);
        }
    };
}
#endif //BIPEDALV1_FANCONTROLLER_H

