//
// Created by divyansh on 7/8/26.
//

#ifndef BIPEDALV1_MPU6050_H
#define BIPEDALV1_MPU6050_H
#include <concepts>

#include "I2C.h"
#include "cmath"

namespace STM32F411::MPU6050 {
    template<typename T>
    concept I2CType = std::same_as<T, I2C1> ||
                      std::same_as<T, I2C2> ||
                      std::same_as<T, I2C3>;

    enum class GyroScale {
        _250 = 0,
        _500,
        _1000,
        _2000,
    };

    enum class AccelScale {
        g2 = 0,
        g4,
        g8,
        g16,
    };

    template<I2CType i2c, GyroScale gyro_scale, AccelScale accel_scale>
    class MPU6050 {
        static constexpr unsigned int buffer_size = 14;
        uint8_t m_buffer[buffer_size] = {};
        volatile float m_pitch = 0.0f;
        volatile float m_roll = 0.0f;
        uint32_t last_update_time = 0;
        volatile bool m_read_in_progress = false;

        struct ZeroOffsets {
            float gx = 0.0f;
            float gy = 0.0f;
            float gz = 0.0f;
            float ax = -0.0384077132f;
            float ay = -0.0255993661f;
        } m_zero_offsets{-1.64820683,-0.403819919,0.0374770537,0.111236326, -0.100898929};

        enum Registers : uint8_t {
            SMPRT_DIV = 25,
            CONFIG = 26,
            GYRO_CONFIG = 27,
            ACCEL_CONFIG,
            INT_PIN_CONFIG = 55,
            INT_ENABLE,
            FIFO_EN = 35,
            ACCEL_XOUT_H = 59,
            ACCEL_XOUT_L,
            ACCEL_YOUT_H,
            ACCEL_YOUT_L,
            ACCEL_ZOUT_H,
            ACCEL_ZOUT_L,
            TEMP_OUT_H,
            TEMP_OUT_L,
            GYRO_XOUT_H = 67,
            GYRO_XOUT_L,
            GYRO_YOUT_H,
            GYRO_YOUT_L,
            GYRO_ZOUT_H,
            GYRO_ZOUT_L,
            PWR_MGMT_1 = 107
        };

        static void dmaReadCompleteCallback(void *ctx) {
            auto *instance = static_cast<MPU6050 *>(ctx);
            instance->m_read_in_progress = false;
            instance->update();
        }

    public:
        static constexpr unsigned int address = 0x68;

        MPU6050() = default;

        void configure(bool enable_interrupt) {
            i2c::enable(true);
            i2c::setCallbacks(dmaReadCompleteCallback, nullptr, this);
            // SET DLPF to delay imu to 2ms
            constexpr uint8_t config_reg_value = (1u << 0);
            i2c::writeRegister(address, Registers::CONFIG, &config_reg_value, 1);

            const uint8_t gyroscope_reg_value = (static_cast<uint8_t>(gyro_scale) << 3);
            const uint8_t accelerometer_reg_value = (static_cast<uint8_t>(accel_scale) << 3);
            i2c::writeRegister(address, Registers::GYRO_CONFIG, &gyroscope_reg_value, 1);
            i2c::writeRegister(address, Registers::ACCEL_CONFIG, &accelerometer_reg_value, 1);


            constexpr uint8_t pwr_mgmt_1_reg_value = (0b001 << 0);
            i2c::writeRegister(address, Registers::PWR_MGMT_1, &pwr_mgmt_1_reg_value, 1);

            constexpr uint8_t smprt_div_reg_value = 1;
            i2c::writeRegister(address, Registers::SMPRT_DIV, &smprt_div_reg_value, 1);

            if (enable_interrupt) {
                constexpr uint8_t int_pin_cfg = (1 << 4);
                i2c::writeRegister(address, Registers::INT_PIN_CONFIG, &int_pin_cfg, 1);

                constexpr uint8_t int_enable = (1 << 0);
                i2c::writeRegister(address, Registers::INT_ENABLE, &int_enable, 1);
            }
            last_update_time = Clock::micros();
        }


        void reset() {
            const uint8_t pwr_mgmt_1_reg = (0b1 << 7) | (0b001 << 0);
            i2c::writeRegister(address, Registers::PWR_MGMT_1, &pwr_mgmt_1_reg, 1);
        }

        void update() {
            if (last_update_time == 0) {
                last_update_time = Clock::micros();
            }
            uint32_t current_time = Clock::micros();
            auto dt = static_cast<float>(current_time - last_update_time) / 1000'000.0f;
            
            // Fixed time constant tau = 0.5s for the filter
            constexpr float tau = 0.5f;
            const float alpha = tau / (tau + dt);
            float ax = getAccelX() - m_zero_offsets.ax;
            float ay = getAccelY() - m_zero_offsets.ay;
            float az = getAccelZ(); // Assuming Z is roughly 1g when flat

            float gx = getGyroX() - m_zero_offsets.gx;
            float gy = getGyroY() - m_zero_offsets.gy;

            constexpr float radian_to_degree = 180.0f / 3.14159265358979323846f;

            float accel_roll = std::atan2(ay, az) * radian_to_degree;
            float accel_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * radian_to_degree;

            m_roll = alpha * (m_roll + gx * dt) + (1.0f - alpha) * accel_roll;
            m_pitch = alpha * (m_pitch + gy * dt) + (1.0f - alpha) * accel_pitch;

            last_update_time = Clock::micros();
        }

        [[nodiscard]] static constexpr float getAccelScaleFactor() {
            return 16384.0f / (1 << static_cast<uint8_t>(accel_scale));
        }

        [[nodiscard]] static constexpr float getGyroScaleFactor() {
            return 131.0f / (1 << static_cast<uint8_t>(gyro_scale));
        }

        void beginRead() {
            if (m_read_in_progress) return; // Skip if previous DMA read hasn't finished
            m_read_in_progress = true;
            if (!i2c::readRegister(address, Registers::ACCEL_XOUT_H, m_buffer, buffer_size, true)) {
                m_read_in_progress = false; // I2C failed before DMA started, reset so next EXTI can retry
            }
        }

        void calibrateGyroscope(const unsigned int sample_size = 4000) {
            float sum_gx = 0, sum_gy = 0, sum_gz = 0;
            for (unsigned int i = 0; i < sample_size; i++) {
                // Force a blocking read for calibration
                i2c::readRegister(address, Registers::ACCEL_XOUT_H, m_buffer, buffer_size, false);

                // Accumulate raw scaled values
                sum_gx += getGyroX();
                sum_gy += getGyroY();
                sum_gz += getGyroZ();
                // Small delay to get fresh samples (at 1kHz sample rate, 1ms is fine)
                Clock::delayMillis(1);
            }

            // Average the samples to find the constant error
            const auto sample_size_f = static_cast<float>(sample_size);
            m_zero_offsets.gx = sum_gx / sample_size_f;
            m_zero_offsets.gy = sum_gy / sample_size_f;
            m_zero_offsets.gz = sum_gz / sample_size_f;

        }
        void calibrateAccelerometer(const unsigned int sample_size = 4000) {
            float sum_ax = 0, sum_ay = 0;

            for (unsigned int i = 0; i < sample_size; i++) {
                // Force a blocking read for calibration
                i2c::readRegister(address, Registers::ACCEL_XOUT_H, m_buffer, buffer_size, false);

                // Accumulate raw scaled values
                sum_ax += getAccelX();
                sum_ay += getAccelY();
                // Small delay to get fresh samples (at 1kHz sample rate, 1ms is fine)
                Clock::delayMillis(1);
            }

            // Average the samples to find the constant error
            const auto sample_size_f = static_cast<float>(sample_size);
            m_zero_offsets.ax = sum_ax / sample_size_f;
            m_zero_offsets.ay = sum_ay / sample_size_f;
        }

        [[nodiscard]] float getAccelX() const {
            return static_cast<int16_t>(m_buffer[0] << 8 | m_buffer[1]) / getAccelScaleFactor();
        }

        [[nodiscard]] float getAccelY() const {
            return static_cast<int16_t>(m_buffer[2] << 8 | m_buffer[3]) / getAccelScaleFactor();
        }

        [[nodiscard]] float getAccelZ() const {
            return static_cast<int16_t>(m_buffer[4] << 8 | m_buffer[5]) / getAccelScaleFactor();
        }

        [[nodiscard]] float getChipTemperature() const {
            return static_cast<int16_t>(m_buffer[6] << 8 | m_buffer[7]);
        }

        [[nodiscard]] float getGyroX() const {
            return (static_cast<int16_t>(m_buffer[8] << 8 | m_buffer[9]) / getGyroScaleFactor())-m_zero_offsets.gx;
        }

        [[nodiscard]] float getGyroY() const {
            return static_cast<int16_t>(m_buffer[10] << 8 | m_buffer[11]) / getGyroScaleFactor()-m_zero_offsets.gy;
        }

        [[nodiscard]] float getGyroZ() const {
            return static_cast<int16_t>(m_buffer[12] << 8 | m_buffer[13]) / getGyroScaleFactor()-m_zero_offsets.gz;
        }

        [[nodiscard]] float getRoll() const { return m_roll; }
        [[nodiscard]] float getPitch() const { return m_pitch; }
    };
}
#endif //BIPEDALV1_MPU6050_H
