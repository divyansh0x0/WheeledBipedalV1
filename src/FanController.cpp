#include "FanController.h"

namespace Biped {

    void FanController::initialize() {
        F411::Pins::B1::enableOutputMode();
        F411::Pins::B1::set(F411::LOW);
    }

    void FanController::setDutyCycle(const float duty_cycle) {
        F411::Pins::B1::set(F411::HIGH);
    }

}
