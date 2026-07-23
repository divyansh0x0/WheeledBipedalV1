//
// Created by divyansh on 7/17/26.
//

#ifndef BIPEDALV1_BUZZER_H
#define BIPEDALV1_BUZZER_H
#include "drivers/GPIO.h"
#include "drivers/PWM.h"

namespace BipedalV1 {
    class Buzzer {
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER4, STM32F411::PWM::TimerChannel::Channel1> m_pwm =
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER4, STM32F411::PWM::TimerChannel::Channel1>();
    public:
        Buzzer()=default;
        Buzzer(Buzzer&& buzzer) = delete;
        Buzzer(Buzzer& buzzer) = delete;

        void initialize() {
            STM32F411::Pins::B6::enableAlternateFunction<STM32F411::Peripherals::TIMER4>();
            m_pwm.enable();
            m_pwm.setFrequency(32000);
            m_pwm.setDutyCycle(0);
        }

        void setDutyCycle(const float duty_cycle) {
            m_pwm.setDutyCycle(duty_cycle);
        }

    };
}
#endif //BIPEDALV1_BUZZER_H
