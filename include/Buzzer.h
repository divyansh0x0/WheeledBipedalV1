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
        unsigned int buzzer_duration_left = 0;
        unsigned int last_time = 0;

    public:
        Buzzer() = default;

        Buzzer(Buzzer &&buzzer) = delete;

        Buzzer(Buzzer &buzzer) = delete;

        void initialize() {
            STM32F411::Pins::B6::enableAlternateFunction<STM32F411::Peripherals::TIMER4>();
            m_pwm.enable();
            m_pwm.setFrequency(25000);
            m_pwm.setDutyCycle(0);
        }

        void setDutyCycle(const float duty_cycle) {
            m_pwm.setDutyCycle(duty_cycle);
        }

        void play(unsigned int duration_ms) {
            this->buzzer_duration_left = duration_ms;
            last_time = STM32F411::Clock::millis();
            update();
        }

        void stop() {
            this->buzzer_duration_left = 0;
            update();
        }

        void update() {
            if (buzzer_duration_left < 10)
                buzzer_duration_left = 0;

            if (buzzer_duration_left == 0) {
                setDutyCycle(0);
                return;
            }

            setDutyCycle(0.5);
            last_time = STM32F411::Clock::millis();
            buzzer_duration_left -= last_time;
        }
    };
}
#endif //BIPEDALV1_BUZZER_H
