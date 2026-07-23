#include "ActuatorManager.h"
#include "BatteryManager.h"
#include "Buzzer.h"
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
STM32F411::MPU6050::MPU6050<STM32F411::I2C1, STM32F411::MPU6050::GyroScale::_250, STM32F411::MPU6050::AccelScale::g2>
mpu6050{};
BipedalV1::BatteryManager<MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE, 133.0f, 33.0f> battery_manager{};
BipedalV1::ActuatorManager actuator_manager{};
BipedalV1::Buzzer  buzzer{};
volatile bool button_is_pressed;

[[noreturn]] int main() {
    using namespace STM32F411;
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C1);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::I2C2);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOC);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOB);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::GPIOA);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::DMA1);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER5);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB2Peripheral::SYSCFG);

    Pins::C13::enableOutputMode();
    Pins::A0::enableInputMode(MemoryMap::GPIORegister::Pull::Up);
    Pins::B10::enableAlternateFunction<Peripherals::SCL2>();
    Pins::B9::enableAlternateFunction<Peripherals::SDA2>();

    Pins::B8::enableAlternateFunction<Peripherals::SCL1>();
    Pins::B7::enableAlternateFunction<Peripherals::SDA1>();

    mpu6050.configure(true);
    battery_manager.initialize();
    actuator_manager.initialize();
    buzzer.initialize();

    mpu6050.beginRead();
    InterruptManager::attachEXTIInterrupt(InterruptManager::EXTILine::Line5, [] { mpu6050.beginRead(); },
                                          InterruptManager::EXTISource::GPIOB, InterruptManager::EXTITrigger::RISING);
    t1 = Clock::millis();
    t2 = Clock::millis();
    bool button_was_pressed = false;
    Pins::C13::set(HIGH); // Turn off LED initially (assuming active low)
    volatile uint32_t button_press_start_time = 0;



    while (true) {
        // --- NON-BLOCKING BUTTON DEBOUNCE & LONG PRESS HANDLING ---
        button_is_pressed = (Pins::A0::getInputState() == LOW);

        if (button_is_pressed) {
            if (!button_was_pressed) {
                // Button was just pressed down
                button_was_pressed = true;
                button_press_start_time = Clock::millis();
            } else {
                // Button is being held down. Check if held for > 1000ms
                if (Clock::millis() - button_press_start_time > 1000) {
                    // Flash LED 4 times to indicate MPU calibration start
                    for (int i = 0; i < 10; ++i) {
                        buzzer.setDutyCycle(50);
                        buzzer.setDutyCycle(50);
                        Clock::delayMillis(200);
                        buzzer.setDutyCycle(0);
                        Clock::delayMillis(200);
                    }
                    Pins::C13::set(LOW); // Keep LED in steady state

                    // Calibrate IMU
                    mpu6050.calibrate();

                    // Reset state to avoid calibrating repeatedly if button is held indefinitely
                    button_was_pressed = false;
                    while(Pins::A0::getInputState() == LOW); // Wait until user lets go
                }
            }
        } else {
            // Button is released
            button_was_pressed = false;
        }

        // If the I2C2 bus is stuck busy (e.g. slave holding SDA low), perform a hardware GPIO recovery
        // if (I2C2::isBusBusy()) {
        //     I2C2::recoverBus<Pins::B10, Pins::B9, Peripherals::SCL2, Peripherals::SDA2>();
        //     encoder.configure(2);
        // }
        //
        // // Read AS5600 encoder on PCA9548A multiplexer channel 2
        // as5600_magnet_status = encoder.readMagnetStatus();
        // if (as5600_magnet_status == AS5600::MagnetStatus::OK) {
        //     // Read RPM directly from the driver (uses EMA filter with alpha=0.1f by default)
        //     // encoder.readRPM(output_rpm);
        //
        //     // Optional: read current angle just for debugging/telemetry
        //     // encoder.readAngle(as5600_angle);
        //
        //     // Calculate base motor RPM and rad/s for control loops
        //     base_motor_rpm = output_rpm * 60.0f; // 60:1 gear ratio
        //     motor_angular_velocity_rad_s = (output_rpm * 6.0f) * (3.14159265359f / 180.0f);
        // }
    }
}
