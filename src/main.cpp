#include "ActuatorManager.h"
#include "BalancePID.h"
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
BipedalV1::Buzzer buzzer{};
volatile bool button_is_pressed;
volatile bool mpu_data_ready = false;
STM32F411::GPIOStatus status = STM32F411::LOW;
BipedalV1::BalancePID balance_pid{1350.0f/10000.0f, 0.0f/10000.0f, 40.0f/10000.0f, 0, 0, 0};
volatile float pid_output = 0.0f;
volatile float BatteryLevel = 0;
float MAX_ROLL_ANGLE = 30.0f;
volatile float gyro_x = 0;
void doPID() {
    const float roll = mpu6050.getRoll();
    gyro_x = mpu6050.getGyroX();
    if (roll > MAX_ROLL_ANGLE || roll < -MAX_ROLL_ANGLE) {
        pid_output = 0.0f;
        balance_pid.reset();
    } else {
        pid_output = balance_pid.getRollPID(mpu6050.getRoll(), mpu6050.getGyroX());
    }
    actuator_manager.move(pid_output, pid_output);
}

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


    actuator_manager.initialize();
    buzzer.initialize();

    // Give MPU6050 and other sensors time to power up and stabilize
    STM32F411::Clock::delayMillis(500);

    mpu6050.configure(true);
    battery_manager.initialize();

    mpu6050.beginRead();
    InterruptManager::attachEXTIInterrupt(InterruptManager::EXTILine::Line5, [] { mpu_data_ready = true; },
                                          InterruptManager::EXTISource::GPIOB, InterruptManager::EXTITrigger::RISING);
    t1 = Clock::millis();
    t2 = Clock::millis();
    bool button_was_pressed = false;
    Pins::C13::set(HIGH); // Turn off LED initially (assuming active low)
    volatile uint32_t button_press_start_time = 0;

    // mpu6050.calibrateGyroscope();

    actuator_manager.initialize();
    actuator_manager.move(0.0f,0.0f);
    battery_manager.initialize();

    while (true) {
        battery_percentage  = battery_manager.getBatteryPercentage();
        BatteryLevel = battery_manager.getBatteryVoltage();
        // --- MPU6050 DMA READ (triggered by EXTI data-ready flag) ---
        if (mpu_data_ready) {
            mpu_data_ready = false;
            mpu6050.beginRead();
        }

        // --- NON-BLOCKING BUTTON DEBOUNCE & LONG PRESS HANDLING ---
        button_is_pressed = (Pins::A0::getInputState() == LOW);

        if (button_is_pressed) {
            if (!button_was_pressed) {
                // Button was just pressed down
                button_was_pressed = true;
                button_press_start_time = Clock::millis();
                Pins::C13::set(LOW); // LED ON while held
            } else if (Clock::millis() - button_press_start_time > 500) {
                // Held for > 1 second — blink LED 5 times
                for (int i = 0; i < 5; i++) {
                    Pins::C13::set(HIGH); // LED OFF (active low)
                    Clock::delayMillis(200);
                    Pins::C13::set(LOW); // LED ON
                    Clock::delayMillis(200);
                }
                Pins::C13::set(HIGH); // LED OFF after blink

                // Calibrate IMU
                mpu6050.calibrateAccelerometer();

                // Wait until user releases the button to avoid re-triggering
                while (Pins::A0::getInputState() == LOW);
                button_was_pressed = false;
            }
        } else {
            if (button_was_pressed) {
                Pins::C13::set(HIGH); // LED OFF on release
            }
            button_was_pressed = false;
        }
        doPID();
    }
}
