//
// Created by divyansh on 7/17/26.
//

#ifndef BIPEDALV1_BUZZER_H
#define BIPEDALV1_BUZZER_H

#include "drivers/GPIO.h"
#include "drivers/PWM.h"
#include "drivers/Clock.h"

namespace BipedalV1 {
    class Buzzer {
        STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER3, STM32F411::PWM::TimerChannel::Channel3> m_pwm =
                STM32F411::PWM::PWM<STM32F411::PWM::Timer::TIMER3, STM32F411::PWM::TimerChannel::Channel3>();
        
        unsigned int start_time = 0;
        unsigned int duration = 0;
        bool is_playing = false;
        float duty_cycle = 0.5f;

    public:
        Buzzer() = default;
        Buzzer(Buzzer &&buzzer) = delete;
        Buzzer(Buzzer &buzzer) = delete;

        void initialize() {
            STM32F411::Pins::B0::enableAlternateFunction<STM32F411::Peripherals::TIMER3>();
            m_pwm.enable();
            m_pwm.setFrequency(4000); // Resonant frequency
            m_pwm.setDutyCycle(0);
        }

        void setDutyCycle(const float new_duty_cycle) {
            this->duty_cycle = new_duty_cycle;
            if (is_playing) {
                m_pwm.setDutyCycle(new_duty_cycle);
            }
        }

        void play(unsigned int duration_ms) {
            this->duration = duration_ms;
            this->start_time = STM32F411::Clock::millis();
            this->is_playing = true;
            m_pwm.setDutyCycle(this->duty_cycle);
        }

        void stop() {
            this->is_playing = false;
            m_pwm.setDutyCycle(0);
        }

        void update() {
            if (!is_playing) return;

            if (STM32F411::Clock::millis() - start_time >= duration) {
                stop();
            }
        }
    };
}
#endif //BIPEDALV1_BUZZER_H
