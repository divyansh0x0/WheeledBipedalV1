//
// Created by divyansh on 7/17/26.
//

#ifndef BIPEDALV1_BATTERYMANAGER_H
#define BIPEDALV1_BATTERYMANAGER_H
#include "drivers/ADC.h"

namespace BipedalV1 {
    template<float min_battery_voltage, float max_battery_voltage, float Resistor1, float Resistor2>
    class BatteryManager {
        STM32F411::ADC::ADC<STM32F411::Pins::B0> adc = STM32F411::ADC::ADC<STM32F411::Pins::B0>();
    public:
        void initialize() {
            adc.enable(STM32F411::ADC::Resolution::VeryHigh);
            adc.enableDMARead();

        }
        constexpr float getADCValue() const {
            return adc.buffer[0];
        }
        constexpr float getBatteryVoltage() const {
            const float pin_voltage = getADCValue() /4095.0f * 3.3f;
            return pin_voltage * Resistor1/Resistor2;
        }
        constexpr float getBatteryPercentage() const {
            float const percentage =  ((getBatteryVoltage() - min_battery_voltage) /
                              (max_battery_voltage - min_battery_voltage)) * 100.0f;
            if (percentage > 100) {
                return 100.0f;
            }
            if (percentage < 0) {
                return 0.0f;
            }
            return percentage;
        }
    };
}
#endif //BIPEDALV1_BATTERYMANAGER_H
