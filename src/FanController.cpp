#include "FanController.h"

namespace BipedalV1 {

    void FanController::initialize() {
        // STM32F411::Pins::B1::enableAlternateFunction<STM32F411::Peripherals::TIMER3>();
        // m_pwm.enable();
        // m_pwm.setDutyCycle(0);
        STM32F411::Pins::B1::enableOutputMode();
        STM32F411::Pins::B1::set(STM32F411::LOW);
    }

    void FanController::setDutyCycle(const float duty_cycle) {
        // m_pwm.setDutyCycle(m_duty_cycle);
        STM32F411::Pins::B1::set(STM32F411::HIGH);
    }

}
