#include "ActuatorManager.h"
#include "drivers/GPIO.h"
#include "drivers/ADC.h"
#include "drivers/Interrupt.h"
#include "drivers/MemoryMap.h"
#include "drivers/MPU6050.h"
#include "drivers/PWM.h"
#include "drivers/AS5600MUX.h"

// Define the real-world battery limits for a 3S LiPo
constexpr float MAX_BATTERY_VOLTAGE = 12.6f;
constexpr float MIN_BATTERY_VOLTAGE = 11.0f;
constexpr float MAX_REFERENCE_VOLTAGE = 3.3f;

// The inverse of your divider (133 / 33) to scale the pin voltage back up to battery voltage
constexpr float VOLTAGE_DIVIDER_RATIO = 133.0f / 33.0f;

volatile float battery_percentage = 0.0f;
volatile float actual_voltage_debug = 0.0f; // Useful to watch in CubeMonitor
volatile unsigned int count = 1000;
volatile unsigned int frequency = 7000;
// 6000 to 7600
volatile float duty = 0;
volatile int16_t gyroX = 0;
volatile int16_t gyroY = 0;
volatile float as5600_angle = 0.0f; // AS5600 angle in degrees (0-360) on MUX channel 2
volatile STM32F411::AS5600::MagnetStatus as5600_magnet_status = STM32F411::AS5600::MagnetStatus::ReadError;
// Magnet status on MUX channel 2
volatile float motor_angular_velocity_rad_s = 0.0f; // Output shaft angular velocity in rad/s
volatile float base_motor_rpm = 0.0f; // Base motor speed in RPM (18000 RPM base / 300 RPM output = 60:1 gear ratio)
volatile float previous_as5600_angle = 0.0f;
volatile uint32_t previous_micros = 0;
volatile float output_rpm;

volatile uint32_t t1;
volatile uint32_t t2;
volatile float speed = 0.0f;
STM32F411::MPU6050::MPU6050<STM32F411::I2C1> mpu6050;

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
    Pins::B10::enableAlternateFunction<Peripherals::SCL2>();
    Pins::B9::enableAlternateFunction<Peripherals::SDA2>();

    Pins::B8::enableAlternateFunction<Peripherals::SCL1>();
    Pins::B7::enableAlternateFunction<Peripherals::SDA1>();
    Pins::B6::enableAlternateFunction<Peripherals::TIMER4>();
    Pins::B1::enableOutputMode();

    Pins::A4::enableOutputMode();

    auto adc = ADC::ADC<Pins::B0>();

    adc.enable(ADC::Resolution::VeryHigh, ADC::SampleTime::Cycles480);
    adc.enableDMARead();

    auto T4C1 = PWM::PWM<PWM::Timer::TIMER4, PWM::TimerChannel::Channel1>();
    T4C1.enable();

    // Initialize I2C2 for the PCA9548A multiplexer + AS5600 encoders
    auto encoder = STM32F411::AS5600::AS5600MUX<STM32F411::I2C2>();
    encoder.configure(2); // mux channel 2

    mpu6050 = MPU6050::MPU6050<I2C1>();
    mpu6050.configure(MPU6050::GyroScale::_500, MPU6050::AccelScale::g2, true);
    mpu6050.beginRead();
    InterruptManager::attachEXTIInterrupt(InterruptManager::EXTILine::Line5, [] { mpu6050.beginRead(); },
                                          InterruptManager::EXTISource::GPIOB, InterruptManager::EXTITrigger::RISING);
    t1 = Clock::millis();
    t2 = Clock::millis();
    T4C1.setDutyCycle(duty);

    BipedalV1::ActuatorManager actuator_manager = {};
    actuator_manager.initialize();

    while (true) {
        T4C1.setFrequency(frequency);
        T4C1.setDutyCycle(duty);

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

        // If the I2C2 bus is stuck busy (e.g. slave holding SDA low), perform a hardware GPIO recovery
        if (I2C2::isBusBusy()) {
            I2C2::recoverBus<Pins::B10, Pins::B9, Peripherals::SCL2, Peripherals::SDA2>();
            encoder.configure(2);
        }

        // Read AS5600 encoder on PCA9548A multiplexer channel 2
        as5600_magnet_status = encoder.readMagnetStatus();
        if (as5600_magnet_status == AS5600::MagnetStatus::OK) {
            // Read RPM directly from the driver (uses EMA filter with alpha=0.1f by default)
            encoder.readRPM(output_rpm);

            // Optional: read current angle just for debugging/telemetry
            encoder.readAngle(as5600_angle);

            // Calculate base motor RPM and rad/s for control loops
            base_motor_rpm = output_rpm * 60.0f; // 60:1 gear ratio
            motor_angular_velocity_rad_s = (output_rpm * 6.0f) * (3.14159265359f / 180.0f);
        }
        if (Clock::millis() - t2 > 1000) {
            duty += 5;
            Pins::C13::toggle();
            t2 = Clock::millis();
        }

        if (duty > 100) {
            duty = 0;
        }
        if (Clock::millis() - t1 > 5000) {
            Pins::B1::set(HIGH);
            t1 = Clock::millis();
        }
       actuator_manager.moveForward(1);

    }
}
