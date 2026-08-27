#include "ActuatorManager.h"
#include "BalancePID.h"
#include "BatteryManager.h"
#include "Buzzer.h"
#include "FanController.h"
#include "drivers/GPIO.h"
#include "drivers/ADC.h"
#include "drivers/Interrupt.h"
#include "drivers/MemoryMap.h"
#include "drivers/MPU6050.h"
#include "drivers/PWM.h"
#include "drivers/AS5600MUX.h"
#include "FanController.h"

// Define the real-world battery limits for a 3S LiPo
constexpr float MAX_BATTERY_VOLTAGE = 12.6f;
constexpr float MIN_BATTERY_VOLTAGE = 10.5f;
constexpr float MAX_REFERENCE_VOLTAGE = 3.3f;

volatile float battery_percentage = 0.0f;
volatile float actual_voltage_debug = 0.0f; // Useful to watch in CubeMonitor
volatile unsigned int count = 1000;
volatile unsigned int frequency = 7000;
// 6000 to 7600
volatile float duty = 0;
volatile float as5600_angle = 0.0f; // AS5600 angle in degrees (0-360) on MUX channel 2

volatile STM32F411::AS5600::MagnetStatus as5600_magnet_status = STM32F411::AS5600::MagnetStatus::ReadError;
volatile uint32_t t1;
volatile float speed = 0.0f;
STM32F411::MPU6050::MPU6050<STM32F411::I2C1, STM32F411::MPU6050::GyroScale::_250, STM32F411::MPU6050::AccelScale::g2>
mpu6050{};
BipedalV1::BatteryManager<MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE, 98.0f, 31.8f> battery_manager{};
static BipedalV1::ActuatorManager actuator_manager{};
static BipedalV1::Buzzer buzzer{};
static volatile bool button_is_pressed;
static volatile bool mpu_data_ready = false;
static STM32F411::GPIOStatus status = STM32F411::LOW;
static BipedalV1::BalancePID balance_pid{1350.0f / 10000.0f, 0.0f / 10000.0f, 40.0f / 10000.0f, 0, 0, 0};

static volatile float pid_output = 0.0f;
volatile float BatteryLevel = 0;


BipedalV1::FanController fan_controller{};

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
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER3);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER5);
    MemoryMap::RCC1->enablePeripheral(MemoryMap::APB2Peripheral::SYSCFG);
    Clock::enable();

    Pins::C13::enableOutputMode();

    Pins::B10::enableAlternateFunction<Peripherals::SCL2>();
    Pins::B9::enableAlternateFunction<Peripherals::SDA2>();

    Pins::B8::enableAlternateFunction<Peripherals::SCL1>();
    Pins::B7::enableAlternateFunction<Peripherals::SDA1>();

    actuator_manager.initialize();
    buzzer.initialize();
    battery_manager.initialize();
    fan_controller.initialize();
    
    // Explicitly enable global interrupts
    asm volatile("cpsie i");

    float speed = -1.0f;
    buzzer.setDutyCycle(0.5f);
    
    t1 = Clock::millis(); // Initialize t1 so the timer math doesn't underflow!
    Clock::delayMillis(1000);
    battery_percentage = battery_manager.getBatteryPercentage();

    if (battery_percentage < 20) {
        buzzer.playTone(BipedalV1::Buzzer::Tones::BATTERY_LOW);
    }
    else {
        buzzer.playTone(BipedalV1::Buzzer::Tones::MANUAL);
        buzzer.stop();
    }
    while (true) {

        battery_percentage = battery_manager.getBatteryPercentage();
        actual_voltage_debug = battery_manager.getBatteryVoltage();
        const auto t2 = Clock::millis();
        
        // Beep 5 times (change < 10 to < 5)
        if (t2 - t1 >= 100) {
            Pins::C13::toggle();
            t1 = Clock::millis();
        }
        
        buzzer.update();
        fan_controller.setDutyCycle(0);
    }
}
