#include "../include/drivers/GPIO.h"
#include "drivers/ADC.h"
#include "drivers/MemoryMap.h"

unsigned int battery_voltage = 0 ;
[[noreturn]] int main() {
    using namespace STM32F411;
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOC);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::DMA2);

    Pins::C13::enableOutputMode();
    Pins::B8::alternateFunction<Peripherals::SCL1>();

    auto adc = ADC::ADC<Pins::B0>();
    adc.enable(ADC::Resolution::VeryHigh, ADC::SampleTime::Cycles480);
    adc.enableDMARead();
    battery_voltage = adc.buffer[0];
    volatile unsigned int i = 0;
    while (true) {
        if (i >  1'000'000 ) {
            Pins::C13::toggle();
            i = 0;
        }
        i++;
    }
}
