//
// Created by divyansh on 7/17/26.
//

#ifndef BIPEDALV1_BATTERYMANAGER_H
#define BIPEDALV1_BATTERYMANAGER_H
#include "drivers/ADC.h"

namespace Biped {
    template<float min_battery_voltage, float max_battery_voltage, float Resistor1, float Resistor2>
    class BatteryManager {
        Biped::ADC::ADC<Biped::Pins::A4> adc = Biped::ADC::ADC<Biped::Pins::A4>();
    public:
        void initialize() {
            adc.enable(Biped::ADC::Resolution::VeryHigh, Biped::ADC::SampleTime::Cycles480);
            adc.enableDMARead();

        }
        [[nodiscard]] float getADCValue() const {
            return adc.buffer[0];
        }
         [[nodiscard]] float getBatteryVoltage() const {
            const float pin_voltage = getADCValue() / 4095.0f * 3.3f;
            return pin_voltage * (Resistor1 + Resistor2) / Resistor2;
        }
         [[nodiscard]] float getBatteryPercentage() const {
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
