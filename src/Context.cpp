//
// Created by divyansh on 8/27/26.
//

#include "Context.h"

#include "ActuatorManager.h"
#include "BatteryManager.h"
#include "Buzzer.h"
#include "FanController.h"
#include "INA219.h"
#include "drivers/MPU6050.h"
#include "drivers/GPIO.h"

// Define mpu6050 in the GLOBAL namespace so CubeMonitor finds it easily without C++ namespace mangling

namespace Biped::Context {
    static Buzzer buzzer{};
    static FanController fan_controller{};
    static INA219Manager ina219_manager{};
    static BatteryManager<10.6f, 12.6f, 98.0f, 31.8f> battery;
    static ActuatorManager actuator_manager{};
    static STM32F411::MPU6050::MPU6050<STM32F411::I2C2, STM32F411::MPU6050::GyroScale::_250,
        STM32F411::MPU6050::AccelScale::g2> mpu6050({
        .gx = 1.0082f, .gy = 7.8047f, .gz = 0.5791, .ax = -0.007324, .ay = -0.01074
    });

    static unsigned int balance_loop_dt_us = 5 * 1'000; // 200hz balancer control loop
    static unsigned int last_balance_loop_time_us = 0;
    static unsigned int last_buzzer_update_time_us = 0;
    static volatile bool mpu_ready = false;

    float getRoll() {
        return mpu6050.getRoll();
    }
    float getGyroY() {
        return mpu6050.getGyroY();
    }
    float getGyroX() {
        return mpu6050.getGyroX();
    }

    float getCurrentHipLeft() {
        return ina219_manager.ina219_data_arr[0].getCurrent();
    }

    float getCurrentHipRight() {
        return ina219_manager.ina219_data_arr[1].getCurrent();
    }
    float getPitch() {
        return mpu6050.getPitch();
    }

    float getVoltageHipLeft() {
        return ina219_manager.ina219_data_arr[0].getVoltage();
    }
    float getVoltageHipRight() {
        return ina219_manager.ina219_data_arr[1].getVoltage();
    }

    static void mpu_irq() {
        mpu_ready = true;
    }
     float getBatteryPercentage() {
        return battery.getBatteryPercentage();
    }
    void initialize() {
        fan_controller.initialize();
        actuator_manager.initialize();
        buzzer.initialize();
        ina219_manager.initialize();
        battery.initialize();

        // Configure PB4 as a digital input so the EXTI hardware can actually read the pin
        STM32F411::Pins::B4::enableInputMode();

        STM32F411::InterruptManager::attachEXTIInterrupt(STM32F411::InterruptManager::EXTILine::Line4, mpu_irq,
                                                         STM32F411::InterruptManager::EXTISource::GPIOB,
                                                         STM32F411::InterruptManager::EXTITrigger::RISING);
        mpu6050.initialize(true);


        // mpu6050.calibrateGyroscope();
        mpu6050.beginRead();
        STM32F411::Clock::delayMillis(200);
        last_balance_loop_time_us = STM32F411::Clock::micros();
    }

     void update() {
        const unsigned int current_time = STM32F411::Clock::micros();
        if (current_time - last_buzzer_update_time_us > 2 * 1'000'000 && battery.getBatteryPercentage() < 10) {
            buzzer.playTone(Buzzer::Tones::BATTERY_LOW);
            last_buzzer_update_time_us = current_time;
        }
        if (mpu_ready) {
            mpu6050.beginRead();
            actuator_manager.enableWheels();
            mpu_ready = false;
        }
        if (current_time - last_balance_loop_time_us >= balance_loop_dt_us) {
            last_balance_loop_time_us = current_time;
        }
        actuator_manager.move(0,0);
        ina219_manager.update();
        mpu6050.update();
        buzzer.update();
    }
}
