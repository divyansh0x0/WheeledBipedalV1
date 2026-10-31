#include "Context.h"

#include "BalancePID.h"
#include "ServoManager.h"
#include "EncoderI2CMux.h"
#include "BatteryManager.h"
#include "Buzzer.h"
#include "FanController.h"
#include "INA219.h"
#include "drivers/MPU6050.h"
#include "drivers/GPIO.h"

namespace Biped::Context {
    constexpr float COM_TARGET_ANGLE = -0.5f;
    constexpr float TARGET_RPM = 0;
    constexpr float MAX_PITCH = 20.0f;

    static PID::PIDVars balance_pid_vars = {
        .kp = 0.0640f,
        .ki = 0.007f,
        .kd = 0.0030f,
        .output_max = 1.0f,
        .target = COM_TARGET_ANGLE,
    };
    static PID::PIDVars speed_pid_vars = {
        .kp = 0.210f,
        .ki = 0.001f,
        .kd = 0.0060f,
        .output_max = 5.0f,
    };
    static BalancePID balance_pid = {balance_pid_vars, speed_pid_vars};
    static Buzzer buzzer{};
    static FanController fan_controller{};
    static INA219Manager ina219_manager{};
    static BatteryManager<10.6f, 12.6f, 98.0f, 31.8f> battery;
    static ServoManager servo_manager{};
    static F411::MPU6050::MPU6050<F411::I2C2, F411::MPU6050::GyroScale::_250,
        F411::MPU6050::AccelScale::g2> mpu6050({
        .gx = 0, .gy =0, .gz =0, .ax = 0, .ay = 0
    });
    static MagneticEncoder::EncoderI2CMux as5600mux{};

    static constexpr unsigned int BALANCE_LOOP_RATE_HZ = 200;
    static constexpr unsigned int BALANCE_LOOP_PERIOD_US = 1'000'000 / BALANCE_LOOP_RATE_HZ;

    static constexpr unsigned int ENCODER_SAMPLE_RATE_HZ = 300;
    static constexpr unsigned int ENCODER_SAMPLE_PERIOD_US = 1'000'000 / ENCODER_SAMPLE_RATE_HZ;

    static constexpr unsigned int ENCODER_DIFFERENTIATE_RATE_HZ = 150;
    static constexpr unsigned int ENCODER_DIFFERENTIATE_PERIOD_US = 1'000'000 / ENCODER_DIFFERENTIATE_RATE_HZ;

    static constexpr unsigned int LOW_BATTERY_ALERT_INTERVAL_US = 2'000'000;
    static constexpr float LOW_BATTERY_THRESHOLD_PERCENT = 10.0f;
    static constexpr unsigned int HIP_DIRECTION_TOGGLE_INTERVAL_US = 500'000;

    static unsigned int last_balance_loop_time_us = 0;
    static unsigned int last_as5600_sample_time_us = 0;
    static unsigned int last_buzzer_update_time_us = 0;
    static unsigned int last_encoder_differentiate_time_us = 0;
    static unsigned int last_hip_update_time_us = 0;

    static float direction = 1.0f;
    static volatile bool mpu_ready = false;
    static unsigned int last_mpu_read_time_us = 0;

    float getRoll() {
        return mpu6050.getRoll();
    }

    float getGyroY() {
        return mpu6050.getGyroY();
    }

    float getGyroX() {
        return mpu6050.getGyroX();
    }

    MagneticEncoder::EncoderI2CMux *getEncoderI2CMux() {
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

    ServoManager *getServoManager() {
        return &servo_manager;
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
    float getLeftWheelRPM() {
        return as5600mux.getWheelLeft()->rpm;
    }
    float getRightWheelRPM() {
        return as5600mux.getWheelRight()->rpm;
    }

    float getLeftWheelAngle() {
        return as5600mux.getWheelLeft()->normalized_angle;

    }
    float getRightWheelAngle() {

        return as5600mux.getWheelRight()->normalized_angle;
    }
    void initialize() {
        F411::Clock::delayMillis(100);

        fan_controller.initialize();
        buzzer.initialize();
        ina219_manager.initialize();
        battery.initialize();

        F411::Pins::B4::enableInputMode(F411::MemoryMap::GPIORegister::Pull::Down);

        mpu6050.initialize(true);

        F411::InterruptManager::attachEXTIInterrupt(F411::InterruptManager::EXTILine::Line4, mpu_irq,
                                                    F411::InterruptManager::EXTISource::GPIOB,
                                                    F411::InterruptManager::EXTITrigger::RISING);

        as5600mux.initialize();
        servo_manager.initialize(100.0f / 2, 173.0f / 2, as5600mux.getWheelLeft(), as5600mux.getWheelRight(),
                                 as5600mux.getHipLeft(), as5600mux.getHipRight());
        mpu6050.beginRead();

        last_balance_loop_time_us = F411::Clock::micros();
        last_mpu_read_time_us = F411::Clock::micros();

        servo_manager.setPWM(0,0);
        F411::Clock::delayMillis(100);
    }

    void balance() {
        const float pitch = mpu6050.getPitch();
        if (fabsf(pitch) > MAX_PITCH) {
            balance_pid.reset();
            servo_manager.setPWM(0,0);
            servo_manager.enableWheels(false);
            return;
        }else {
            servo_manager.enableWheels(true);
        }
        balance_pid.setTargetSpeed(TARGET_RPM);
        const float angle_offset = balance_pid.getTargetAngle(
            servo_manager.getLeftWheelRPM(), servo_manager.getRightWheelRPM());
        balance_pid.setTargetAngle(COM_TARGET_ANGLE + angle_offset);

        const float balance_speed = balance_pid.getBalancePIDMotorSpeed(pitch, mpu6050.getGyroY());
        servo_manager.setPWM(-balance_speed, -balance_speed);
    }

    void update() {
        const unsigned int current_time = F411::Clock::micros();

        if (mpu_ready) {
            mpu_ready = false;
            mpu6050.beginRead();
            last_mpu_read_time_us = current_time;
        } else if (current_time - last_mpu_read_time_us > 10'000) {
            mpu6050.beginRead();
            last_mpu_read_time_us = current_time;
        }
        if (current_time - last_buzzer_update_time_us > LOW_BATTERY_ALERT_INTERVAL_US && battery.getBatteryPercentage()
            < LOW_BATTERY_THRESHOLD_PERCENT) {
            buzzer.playTone(Buzzer::Tones::BATTERY_LOW);
            last_buzzer_update_time_us = current_time;
        } else if (battery.getBatteryPercentage() > LOW_BATTERY_THRESHOLD_PERCENT) {
            buzzer.stop();
        }

        if (current_time - last_balance_loop_time_us >= BALANCE_LOOP_PERIOD_US) {
            last_balance_loop_time_us = current_time;
            balance();
        }

        if (current_time - last_as5600_sample_time_us >= ENCODER_SAMPLE_PERIOD_US) {
            last_as5600_sample_time_us = current_time;
            as5600mux.update();
        }

        if (current_time - last_encoder_differentiate_time_us >= ENCODER_DIFFERENTIATE_PERIOD_US) {
            last_encoder_differentiate_time_us = current_time;
            as5600mux.differentiate();
        }

        ina219_manager.update();
        buzzer.update();
    }
}