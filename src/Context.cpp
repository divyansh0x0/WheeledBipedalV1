//
// Created by divyansh on 8/27/26.
//

#include "Context.h"

#include "ServoManager.h"
#include "AS5600MUX.h"
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
    static ServoManager servo_manager{};
    static MPU6050::MPU6050<I2C2, MPU6050::GyroScale::_250,
        MPU6050::AccelScale::g2> mpu6050({
        .gx = 1.0082f, .gy = 7.8047f, .gz = 0.5791f, .ax = -0.007324f, .ay = -0.01074f
    });
    static AS5600::AS5600MUX as5600mux{};

    static unsigned int balance_loop_dt_us = 5 * 1'000; // 200hz balancer control loop
    static unsigned int last_balance_loop_time_us = 0;
    static unsigned int last_buzzer_update_time_us = 0;
    static unsigned int last_hip_update_time_us = 0;

    static float direction = 1.0f;
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

    AS5600::AS5600MUX *getAS5600MUX() {
        return &as5600mux;
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

    float getBatteryVoltage() {
        return battery.getBatteryVoltage();
    }

    void initialize() {
        fan_controller.initialize();
        buzzer.initialize();
        ina219_manager.initialize();
        battery.initialize();

        Biped::Pins::B4::enableInputMode();

        Biped::InterruptManager::attachEXTIInterrupt(Biped::InterruptManager::EXTILine::Line4, mpu_irq,
                                                     Biped::InterruptManager::EXTISource::GPIOB,
                                                     Biped::InterruptManager::EXTITrigger::RISING);
        mpu6050.initialize(true);
        as5600mux.initialize();
        servo_manager.initialize(100.0f / 2, 173.0f / 2, as5600mux.getWheelLeft(), as5600mux.getWheelRight(),
                                    as5600mux.getHipLeft(), as5600mux.getHipRight());
        mpu6050.beginRead();

        last_balance_loop_time_us = Biped::Clock::micros();

        buzzer.playTone(Buzzer::Tones::BEEP_BEEP);
        Biped::Clock::delayMillis(200);
    }

    void update() {
        const unsigned int current_time = Biped::Clock::micros();
        // if (current_time - last_buzzer_update_time_us > 2 * 1'000'000 && battery.getBatteryPercentage() < 10) {
        //     buzzer.playTone(Buzzer::Tones::BATTERY_LOW);
        //     last_buzzer_update_time_us = current_time;
        // }
        // else if (battery.getBatteryPercentage() > 10){
        //     buzzer.stop();
        // }
        if (mpu_ready) {
            mpu6050.beginRead();
            servo_manager.enableWheels();
            mpu_ready = false;
        }
        if (current_time - last_balance_loop_time_us >= balance_loop_dt_us) {
            last_balance_loop_time_us = current_time;
        }
        if (current_time - last_hip_update_time_us >= 500'000) {
            last_hip_update_time_us = current_time;
            direction *= -1;
        }

        servo_manager.setWheelRPM(0, 0);
        ina219_manager.update();
        mpu6050.update();
        buzzer.update();
    }
}
