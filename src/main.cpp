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
#include "../include/AS5600MUX.h"
#include "FanController.h"

static inline volatile float current[2] = {};
static inline volatile float voltage[2] = {};

static inline volatile float battery_percentage;

[[noreturn]] int main() {
    using namespace STM32F411;
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C1);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C3);
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

    Pins::B6::enableAlternateFunction<Peripherals::SCL1>();
    Pins::B7::enableAlternateFunction<Peripherals::SDA1>();

    Pins::B8::enableAlternateFunction<Peripherals::SDA3>();
    Pins::A8::enableAlternateFunction<Peripherals::SCL3>();

    Biped::Context::initialize();

    while (true) {
        Biped::Context::update();
        current[0] = Biped::Context::getCurrentHipLeft();
        current[1] = Biped::Context::getCurrentHipRight();
        voltage[0] = Biped::Context::getVoltageHipLeft();
        voltage[1] = Biped::Context::getVoltageHipRight();
        battery_percentage = Biped::Context::getBatteryPercentage();
    }
}
