//
// Created by divyansh on 8/27/26.
//

#include "Context.h"

#include "ActuatorManager.h"
#include "BatteryManager.h"
#include "Buzzer.h"
#include "FanController.h"
#include "drivers/MPU6050.h"
#include "drivers/GPIO.h"

// Define mpu6050 in the GLOBAL namespace so CubeMonitor finds it easily without C++ namespace mangling

namespace Biped::Context {
    static Buzzer buzzer{};
    static FanController fan_controller{};
    static BatteryManager<10.6f, 12.6f, 98.0f, 31.8f> battery;
    static ActuatorManager actuator_manager{};


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

    float getPitch() {
        return mpu6050.getPitch();
    }

    static void mpu_irq() {
        mpu_ready = true;
    }
    [[noreturn]] void initialize() {
        fan_controller.initialize();
        actuator_manager.initialize();
        buzzer.initialize();
        battery.initialize();

        // Configure PB4 as a digital input so the EXTI hardware can actually read the pin
        STM32F411::Pins::B4::enableInputMode();

        STM32F411::InterruptManager::attachEXTIInterrupt(STM32F411::InterruptManager::EXTILine::Line4, mpu_irq,
                                                         STM32F411::InterruptManager::EXTISource::GPIOB,
                                                         STM32F411::InterruptManager::EXTITrigger::RISING);
        mpu6050.initialize(true);


        mpu6050.calibrateGyroscope();
        mpu6050.beginRead();
        STM32F411::Clock::delayMillis(200);
        last_balance_loop_time_us = STM32F411::Clock::micros();
        while (true) {
            update();
        }
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
        mpu6050.update();
        buzzer.update();
    }
}
