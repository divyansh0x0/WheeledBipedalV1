#include "ActuatorManager.h"
#include "BalancePID.h"
#include "BatteryManager.h"
#include "Buzzer.h"
#include "Context.h"
#include "FanController.h"
#include "drivers/GPIO.h"
#include "drivers/ADC.h"
#include "drivers/Interrupt.h"
#include "drivers/MemoryMap.h"
#include "drivers/MPU6050.h"
#include "drivers/PWM.h"
#include "drivers/AS5600MUX.h"
#include "FanController.h"


[[noreturn]] int main() {
    using namespace STM32F411;
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C1);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOC);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOB);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOA);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::DMA1);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER5);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER3);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB2Peripheral::SYSCFG);
    Clock::enable();

    Pins::C13::enableOutputMode();
    Pins::B10::enableAlternateFunction<Peripherals::SCL2>();
    Pins::B9::enableAlternateFunction<Peripherals::SDA2>();

    Pins::B8::enableAlternateFunction<Peripherals::SCL1>();
    Pins::B7::enableAlternateFunction<Peripherals::SDA1>();

    Biped::Context::initialize();
}
