//
// Created by divyansh on 9/10/26.
//

#include "EncoderI2CMux.h"
#include "drivers/GPIO.h"

static float normalize_angle(float angle) {
    return angle > 180 ? angle - 360 : angle < -180 ? angle + 360 : angle;
}

static void encoder_read_callback(void *ctx) {
    const auto mux = reinterpret_cast<Biped::MagneticEncoder::EncoderI2CMux *>(ctx);
    Biped::MagneticEncoder::Encoder *state = mux->getCurrentEncoder();
    const float max_angle_binary = state->type == Biped::MagneticEncoder::Encoder::EncoderType::AS5600
                                       ? 4096.0f
                                       : 16384.0f;
    const float new_angle =
            static_cast<float>(static_cast<uint16_t>(state->buffer[0] << 8 | state->buffer[1])) *
            360.0f
            / max_angle_binary;


    const unsigned int current_time = F411::Clock::micros();
    const float angle_change = normalize_angle(new_angle - state->raw_angle);
    const unsigned int dt = current_time - state->last_read_time_us;
    const float rpm = (angle_change * 1e6f/60.0f)/static_cast<float>(dt);
    // Accumulate continuous angle to prevent wrap-around across the history window
    state->raw_angle = new_angle;
    state->rpm = rpm;
    // Insert current point into history
    state->normalized_angle = normalize_angle(state->raw_angle - state->reference_angle.value());
    state->last_read_time_us = current_time;
    if (mux->changeChannelDMA())
        mux->updateDataDMA();
}

// static void mux_write_callback(void *ctx) {
//     const auto as5600mux = reinterpret_cast<Biped::AS5600::EncoderI2CMux *>(ctx);
//     as5600mux->updateDataDMA();
// }
static unsigned int AS5600_ADDR = 0x36;
static unsigned int MT6701_ADDR = 0x06;

enum class AS5600Registers : uint8_t {
    ZMCO = 0x0,
    ZPOS_L = 0x01,
    ZPOS_H = 0x02,
    MPOS_L = 0x03,
    MPOS_H = 0x04,
    MANG_L = 0x05,
    MANG_H = 0x06,
    CONF_H = 0x07,
    CONF_L = 0x08,
    STATUS = 0x0B,
    RAW_ANGLE_H = 0x0C, // 12-bit raw angle [11:8]
    RAW_ANGLE_L = 0x0D, // 12-bit raw angle  [7:0]
    ANGLE_H = 0x0E, // 12-bit filtered angle [11:8]
    ANGLE_L = 0x0F, // 12-bit filtered angle  [7:0]
    AGC = 0x1A,
    MAGNITUDE_H = 0x1B,
    MAGNITUDE_L = 0x1C,
};

enum class MT6701Registers : uint8_t {
    ANGLE_MSB = 0x03,
    ANGLE_LSB = 0x04,
    PUSH_BTN = 0x05,
};

namespace Biped::MagneticEncoder {
    void Encoder::readMagnetStatus() {
        if (type == EncoderType::MT6701) {
            this->status.reset();
            return;
        }

        // AS5600 read
        uint8_t temp = 0;
        if (!i2c::readRegister(AS5600_ADDR, static_cast<uint8_t>(AS5600Registers::STATUS), &temp, 1)) {
            i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
            this->status = MagnetStatus::ReadError;
            return;
        }


        const bool md = temp & (1 << 5);
        const bool ml = temp & (1 << 4);
        const bool mh = temp & (1 << 3);

        if (!md)
            this->status = MagnetStatus::NotDetected;
        if (ml)
            this->status = MagnetStatus::TooWeak;
        if (mh)
            this->status = MagnetStatus::TooStrong;
        this->status = MagnetStatus::OK;
    }

    void Encoder::updateAngle() {
        if (type == EncoderType::MT6701) {
            if (!i2c::readRegister(MT6701_ADDR, static_cast<uint8_t>(MT6701Registers::ANGLE_MSB), this->buffer, 2)) {
                i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
                this->status = MagnetStatus::ReadError;
                return;
            }
        } else {
            //AS5600 read
            if (!i2c::readRegister(AS5600_ADDR, static_cast<uint8_t>(AS5600Registers::RAW_ANGLE_H), this->buffer, 2)) {
                i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
                this->status = MagnetStatus::ReadError;
                return;
            }
        }

        const float max_angle_bit = this->type == EncoderType::AS5600 ? 4096.0f : 16384.0f;
        const float new_angle = normalize_angle(
            static_cast<float>(static_cast<uint16_t>(this->buffer[0] << 8 | this->buffer[1])) *
            360.0f
            / max_angle_bit);


        const unsigned int current_time = F411::Clock::micros();
        if (this->last_read_time_us == 0) {
            this->last_read_time_us = current_time;
            this->raw_angle = new_angle;
        }

        this->raw_angle = new_angle;

        this->normalized_angle = normalize_angle(this->raw_angle - this->reference_angle.value());
        this->last_read_time_us = current_time;
    }

    bool EncoderI2CMux::changeChannel() {
        this->active_as5600_index = (this->active_as5600_index + 1) % this->as5600_count;
        this->active_channel = this->as5600_states[this->active_as5600_index].mux_index;
        bool success = i2c::writeRegister(PCA9548A_ADDR, static_cast<uint8_t>(0b1 << this->active_channel), nullptr, 0,
                                          false);
        if (!success) {
            i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
        }
        return success;
    }

    Encoder *EncoderI2CMux::getCurrentEncoder() {
        return &as5600_states[this->active_as5600_index];
    }

    bool EncoderI2CMux::changeChannelDMA() {
        this->active_as5600_index = (this->active_as5600_index + 1) % this->as5600_count;
        this->active_channel = this->as5600_states[this->active_as5600_index].mux_index;
        auto channel_mask = static_cast<uint8_t>(0b1 << this->active_channel);
        if (!i2c::writeRegister(PCA9548A_ADDR, channel_mask, nullptr, 0, false)) {
            i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
        }
        return this->active_channel != 0;
    }

    void EncoderI2CMux::updateDataDMA() {
        for (unsigned int i = 0; i < this->as5600_count; i++) {
            Encoder *current_encoder = this->getCurrentEncoder();
            if (current_encoder->type == Encoder::EncoderType::MT6701 && i2c::readRegister(
                    MT6701_ADDR, static_cast<uint8_t>(MT6701Registers::ANGLE_MSB),
                    current_encoder->buffer, 2, true)) {
                return; // DMA started successfully, callback will handle the rest
            }
            if (current_encoder->type == Encoder::EncoderType::MT6701 && i2c::readRegister(
                    AS5600_ADDR, static_cast<uint8_t>(AS5600Registers::RAW_ANGLE_H),
                    current_encoder->buffer, 2, true)) {
                return; // DMA started successfully, callback will handle the rest
            }

            // Failed to update (NACK). Mark error, recover bus, and try the next channel.
            current_encoder->status = MagnetStatus::ReadError;
            i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
            if (!this->changeChannelDMA()) {
                return; // Reached channel 0
            }
        }
    }

    void EncoderI2CMux::update() {
        this->updateDataDMA();
    }

    bool EncoderI2CMux::initialize() {
        i2c::enable(true);
        i2c::setCallbacks(encoder_read_callback, nullptr, this);
        for (unsigned int i = 0; i < 20; i++) {
            for (unsigned int j = 0; j < this->as5600_count; j++) {
                changeChannel();
                readMagnetStatus();
                updateAngles();
            }
        }
        return true;
    }

    void EncoderI2CMux::updateAngles() {
        unsigned int channel_mask = 0b1 << this->active_channel;
        Encoder *state = &this->as5600_states[this->active_as5600_index];
        if (!i2c::writeRegister(PCA9548A_ADDR, channel_mask, nullptr, 0, false)) {
            i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
            state->status = MagnetStatus::ReadError;
            return;
        }
        state->updateAngle();
    }

    void EncoderI2CMux::readMagnetStatus() {
        const unsigned int channel_mask = 0b1 << this->active_channel;
        Encoder *state = &this->as5600_states[this->active_as5600_index];
        if (!i2c::writeRegister(PCA9548A_ADDR, channel_mask, nullptr, 0, false)) {
            i2c::recoverBus<F411::Pins::B6, F411::Pins::B7, F411::Peripherals::SCL1, F411::Peripherals::SDA1>();
            state->status = MagnetStatus::ReadError;
            return;
        }
        state->readMagnetStatus();
    }
}
