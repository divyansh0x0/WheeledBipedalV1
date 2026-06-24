#include "drivers/GPIO.h"
#include "drivers/MemoryMap.h"

#define seconds 2
[[noreturn]] int main() {
    STM32::MemoryMap::RCC1->enablePeripheral(STM32::MemoryMap::AHB1Peripheral::GPIOC);
    STM32::Pins::C13::enable();

    volatile unsigned int i = 0;
    while (true) {
        if (i >  1'000'000 ) {
            STM32::Pins::C13::toggle();
            i = 0;
        }
        i++;
    }
}
