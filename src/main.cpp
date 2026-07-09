#include "drivers/GPIO.h"
#include "drivers/ADC.h"
#include "drivers/Interrupt.h"
#include "drivers/MemoryMap.h"
#include "drivers/MPU6050.h"
#include "drivers/PWM.h"

// Define the real-world battery limits for a 3S LiPo
constexpr float MAX_BATTERY_VOLTAGE = 12.6f;
constexpr float MIN_BATTERY_VOLTAGE = 11.0f;
constexpr float MAX_REFERENCE_VOLTAGE = 2.9f;

// The inverse of your divider (133 / 33) to scale the pin voltage back up to battery voltage
constexpr float VOLTAGE_DIVIDER_RATIO = 133.0f / 33.0f;

volatile float battery_percentage = 0.0f;
volatile float actual_voltage_debug = 0.0f; // Useful to watch in CubeMonitor
volatile unsigned int count = 1000;
volatile unsigned int frequency = 4000;
volatile unsigned int ARR = 1000;
volatile unsigned int PSC = 1000;
volatile int16_t gyroX = 0;
volatile int16_t gyroY = 0;
volatile uint32_t t1;
volatile uint32_t t2;
STM32F411::MPU6050::MPU6050<STM32F411::I2C1> mpu6050;
STM32F411::MemoryMap::DMAStream *dma_stream;

[[noreturn]] int main() {
    using namespace STM32F411;
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C1);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOC);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOB);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::DMA1);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER5);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB2Peripheral::SYSCFG);

    Pins::C13::enableOutputMode();
    Pins::B8::enableAlternateFunction<Peripherals::SCL1>();
    Pins::B7::enableAlternateFunction<Peripherals::SDA1>();
    auto adc = ADC::ADC<Pins::B0>();
    adc.enable(ADC::Resolution::VeryHigh, ADC::SampleTime::Cycles480);
    adc.enableDMARead();

    Pins::B6::enableAlternateFunction<Peripherals::TIMER4>();
    auto T4C2 = PWM::PWM<PWM::Timer::TIMER4, PWM::TimerChannel::Channel1>();
    T4C2.enable();
    T4C2.setDutyCycle(50);


    mpu6050 =  MPU6050::MPU6050<I2C1>();
    mpu6050.configure(MPU6050::GyroScale::_500, MPU6050::AccelScale::g2, true);
    mpu6050.beginRead();
    InterruptManager::attachEXTIInterrupt(InterruptManager::EXTILine::Line5, []{mpu6050.beginRead();}, InterruptManager::EXTISource::GPIOB, InterruptManager::EXTITrigger::RISING);
    t1 = Clock::millis();
    t2 = Clock::millis();
    while (true) {
        T4C2.setFrequency(frequency);
        count = MemoryMap::TIMER4->CNT;
        // 1. Calculate voltage at the physical pin (0-4095 range for 12-bit)
        const float pin_voltage = (static_cast<float>(adc.buffer[0]) / 4095.0f) * MAX_REFERENCE_VOLTAGE;

        // 2. Scale back up to the real battery voltage
        actual_voltage_debug = pin_voltage * VOLTAGE_DIVIDER_RATIO;

        // 3. Map to 0-100% based on your LiPo limits
        battery_percentage = ((actual_voltage_debug - MIN_BATTERY_VOLTAGE) /
                              (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0f;

        // 4. Clamp the values so it doesn't read 110% hot off the charger or -5% when dead
        if (battery_percentage > 100.0f) battery_percentage = 100.0f;
        if (battery_percentage < 0.0f) battery_percentage = 0.0f;

        // A constant 4 kHz frequency produces a sharp sound for most passive buzzers
        PSC = MemoryMap::TIMER4->PSC;
        ARR = MemoryMap::TIMER4->ARR;
        gyroX = mpu6050.getGyroX();
        gyroY = mpu6050.getGyroY();
        if (Clock::millis() - t2 > 1000) {
            frequency += 20;
            Pins::C13::toggle();
            t2 = Clock::millis();
        }
        if (frequency > 15000) {
            frequency = 3000;
        }
    }
}
