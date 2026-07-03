#include "../include/drivers/GPIO.h"
#include "drivers/ADC.h"
#include "drivers/MemoryMap.h"
#include "drivers/PWM .h"

// Define the real-world battery limits for a 3S LiPo
constexpr float MAX_BATTERY_VOLTAGE = 12.6f;
constexpr float MIN_BATTERY_VOLTAGE = 11.0f;
constexpr float MAX_REFERENCE_VOLTAGE = 3.3f;

// The inverse of your divider (133 / 33) to scale the pin voltage back up to battery voltage
constexpr float VOLTAGE_DIVIDER_RATIO = 133.0f / 33.0f;

volatile float battery_percentage = 0.0f;
volatile float actual_voltage_debug = 0.0f; // Useful to watch in CubeMonitor
volatile unsigned int count = 1000;
   volatile unsigned int frequency = 1000;
   volatile unsigned int ARR = 1000;
   volatile unsigned int CNT = 1000;
   volatile unsigned int PSC = 1000;

volatile uint32_t t1;
volatile uint32_t t2;

[[noreturn]] int main() {
    using namespace STM32F411;
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOC);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOB);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::DMA2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER5);

    Pins::C13::enableOutputMode();
    auto adc = ADC::ADC<Pins::B0>();
    adc.enable(ADC::Resolution::VeryHigh, ADC::SampleTime::Cycles480);
    adc.enableDMARead();

    Pins::B7::enableAlternateFunction<Peripherals::TIMER4>();
        PWM::PWM<PWM::Timer::TIMER4, PWM::TimerChannel::Channel2>::setFrequency(frequency);
    PWM::PWM<PWM::Timer::TIMER4, PWM::TimerChannel::Channel2>::enable();
    PWM::PWM<PWM::Timer::TIMER4, PWM::TimerChannel::Channel2>::setDutyCycle(50);
    t1 = Clock::millis();
    t2 = Clock::millis();
    while (true) {
        count = MemoryMap::TIMER4->CNT;
        // 1. Calculate voltage at the physical pin (0-4095 range for 12-bit)
        float pin_voltage = (static_cast<float>(adc.buffer[0]) / 4095.0f) * MAX_REFERENCE_VOLTAGE;

        // 2. Scale back up to the real battery voltage
        actual_voltage_debug = pin_voltage * VOLTAGE_DIVIDER_RATIO;

        // 3. Map to 0-100% based on your LiPo limits
        battery_percentage = ((actual_voltage_debug - MIN_BATTERY_VOLTAGE) /
                              (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0f;

        // 4. Clamp the values so it doesn't read 110% hot off the charger or -5% when dead
        if (battery_percentage > 100.0f) battery_percentage = 100.0f;
        if (battery_percentage < 0.0f) battery_percentage = 0.0f;

        if (Clock::millis() - t1 > 20) {
            frequency += Clock::millis() - t1;
            t1 = Clock::millis();
        }
        if (frequency > 10000) {
            frequency = 100;
        }
        PWM::PWM<PWM::Timer::TIMER4, PWM::TimerChannel::Channel2>::setFrequency(frequency);
        PSC = MemoryMap::TIMER4->PSC;
        ARR = MemoryMap::TIMER4->ARR;
        CNT = MemoryMap::TIMER4->CNT;
        if (Clock::millis() - t2 > 1000) {
            Pins::C13::toggle();
            t2 = Clock::millis();
        }
    }
}
