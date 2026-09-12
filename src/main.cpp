#include "Context.h"
#include "ServoManager.h"
#include "drivers/Clock.h"
#include "drivers/GPIO.h"
#include "drivers/MemoryMap.h"


static inline volatile float current[2] = {};
static inline volatile float voltage[2] = {};

static inline volatile float battery_percentage;
static inline volatile float battery_voltage;
static inline volatile float roll;
static inline volatile float pitch;
static Biped::AS5600::AS5600MUX *mux = nullptr;;
static Biped::ServoManager *servo_manager = nullptr;;
static volatile unsigned int control_loop_rate_hz = 0;
static volatile float right_wheel_rpm = 0;
static volatile float left_wheel_rpm= 0;
[[noreturn]] int main() {
    using namespace Biped;
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

    Context::initialize();
    mux = Context::getAS5600MUX();
    servo_manager = Context::getServoManager();

    servo_manager->enableWheels();
    constexpr float rpm = 20;
    servo_manager->setLeftWheelRPM(0);
    servo_manager->setLeftWheelRPM(0);
    servo_manager->setLeftWheelRPM(rpm);
    servo_manager->setRightWheelRPM(rpm);
    mux->update();
    unsigned int t1 = Clock::millis();
    unsigned int count = 0;
    while (true) {
        Context::update();
        current[0] = Context::getCurrentHipLeft();
        current[1] = Context::getCurrentHipRight();
        voltage[0] = Context::getVoltageHipLeft();
        voltage[1] = Context::getVoltageHipRight();
        battery_percentage = Context::getBatteryPercentage();
        battery_voltage = Context::getBatteryVoltage();
        roll = Context::getRoll();
        pitch = Context::getPitch();
        count++;

        if (Clock::millis() - t1 > 1000) {
            control_loop_rate_hz = count;
            count = 0;
            t1 = Clock::millis();
        }
        right_wheel_rpm = servo_manager->getRightWheelRPM();
        left_wheel_rpm = servo_manager->getLeftWheelRPM();
    }
}
