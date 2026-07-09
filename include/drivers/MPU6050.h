//
// Created by divyansh on 7/8/26.
//

#ifndef BIPEDALV1_MPU6050_H
#define BIPEDALV1_MPU6050_H
#include <concepts>

#include "I2C.h"

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

    template<I2CType i2c>
    class MPU6050 {
        static constexpr unsigned int buffer_size = 14;
        uint8_t buffer[buffer_size] = {};

        enum Registers : uint8_t {
            SMPRT_DIV=25,
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

    public:
        static constexpr unsigned int address = 0x68;

        MPU6050() = default;

        void configure(GyroScale gyro_scale, AccelScale accel_scale, bool enable_interrupt) {
            i2c::enable(true);
            // SET DLPF to delay imu to 2ms
            const uint8_t config_reg_value = (1u << 0);
            i2c::writeRegister(address, Registers::CONFIG, &config_reg_value, 1);

            const uint8_t gyroscope_reg_value = (static_cast<uint8_t>(gyro_scale) << 3);
            const uint8_t accelerometer_reg_value = (static_cast<uint8_t>(accel_scale) << 3);
            i2c::writeRegister(address, Registers::GYRO_CONFIG, &gyroscope_reg_value, 1);
            i2c::writeRegister(address, Registers::ACCEL_CONFIG, &accelerometer_reg_value, 1);


            const uint8_t pwr_mgmt_1_reg_value = (0b001 << 0);
            i2c::writeRegister(address, Registers::PWR_MGMT_1, &pwr_mgmt_1_reg_value, 1);

            const uint8_t smprt_div_reg_value = 1;
            i2c::writeRegister(address, Registers::SMPRT_DIV, &smprt_div_reg_value, 1);

            if (enable_interrupt) {
                const uint8_t int_pin_cfg = (1 << 4);
                i2c::writeRegister(address, Registers::INT_PIN_CONFIG, &int_pin_cfg, 1);

                const uint8_t int_enable = (1 << 0);
                i2c::writeRegister(address, Registers::INT_ENABLE, &int_enable, 1);
            }
        }

        void reset() {
            const uint8_t pwr_mgmt_1_reg = (0b1 << 7) | (0b001 << 0);
            i2c::writeRegister(address, Registers::PWR_MGMT_1, &pwr_mgmt_1_reg, 1);
        }

        void beginRead() {
            auto stream_id = MemoryMap::DMA1->STREAMS[0].isEnabled()
                     ? 5
                     : 0;
            MemoryMap::DMAStream* dma_stream = &MemoryMap::DMA1->STREAMS[stream_id];
            InterruptManager::attachDMAInterrupt(static_cast<InterruptManager::Stream>(stream_id), []{
                I2C1::finishDMARead();
            });
            dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH1);
            i2c::readRegister(address, Registers::ACCEL_XOUT_H, buffer, buffer_size, dma_stream);
        }

        [[nodiscard]] int16_t getAccelX() const {
            return buffer[0] << 8 | buffer[1];
        }

        [[nodiscard]] int16_t getAccelY() const {
            return buffer[2] << 8 | buffer[3];
        }

        [[nodiscard]] int16_t getAccelZ() const {
            return buffer[4] << 8 | buffer[5];
        }

        [[nodiscard]] int16_t getChipTemperature() const {
            return buffer[6] << 8 | buffer[7];
        }

        [[nodiscard]] int16_t getGyroX() const {
            return buffer[8] << 8 | buffer[9];
        }

        [[nodiscard]] int16_t getGyroY() const {
            return buffer[10] << 8 | buffer[11];
        }

        [[nodiscard]] int16_t getGyroZ() const {
            return buffer[12] << 8 | buffer[13];
        }
    };
}
#endif //BIPEDALV1_MPU6050_H
