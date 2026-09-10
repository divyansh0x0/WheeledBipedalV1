#include "FanController.h"

namespace Biped {

    void FanController::initialize() {
        // STM32F411::Pins::B1::enableAlternateFunction<STM32F411::Peripherals::TIMER3>();
        // m_pwm.enable();
        // m_pwm.setDutyCycle(0);
        Biped::Pins::B1::enableOutputMode();
        Biped::Pins::B1::set(Biped::LOW);
    }

    void FanController::setDutyCycle(const float duty_cycle) {
        // m_pwm.setDutyCycle(m_duty_cycle);
        Biped::Pins::B1::set(Biped::HIGH);
    }

}
