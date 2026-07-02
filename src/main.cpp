#include "../include/drivers/GPIO.h"
#include "drivers/MemoryMap.h"

[[noreturn]] int main() {
    using namespace STM32F411;
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOC);
    Pins::C13::enableOutputMode();

    Pins::B8::alternateFunction<Peripherals::SCL1>();
    volatile unsigned int i = 0;
    while (true) {
        if (i >  1'000'000 ) {
            Pins::C13::toggle();
            i = 0;
        }
        i++;
    }
}
