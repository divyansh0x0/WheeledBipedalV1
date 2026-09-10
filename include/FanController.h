//
// Created by divyansh on 8/26/26.
//


#ifndef BIPEDALV1_FANCONTROLLER_H
#define BIPEDALV1_FANCONTROLLER_H
#include "drivers/GPIO.h"
#include "drivers/PWM.h"

namespace Biped {
    class FanController {
        Biped::PWM::PWM<Biped::PWM::Timer::TIMER3, Biped::PWM::TimerChannel::Channel4> m_pwm =
                Biped::PWM::PWM<Biped::PWM::Timer::TIMER3, Biped::PWM::TimerChannel::Channel4>();

    public:
        FanController() = default;

        FanController(FanController &&buzzer) = delete;

        FanController(FanController &buzzer) = delete;

        void initialize();
        void setDutyCycle(const float duty_cycle);
    };
}
#endif //BIPEDALV1_FANCONTROLLER_H

