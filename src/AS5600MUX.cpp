//
// Created by divyansh on 9/10/26.
//

#include "AS5600MUX.h"
#include "drivers/GPIO.h"

static void encoder_read_callback(void *ctx) {
    const auto as5600mux = reinterpret_cast<Biped::AS5600::AS5600MUX *>(ctx);
    Biped::AS5600::AS5600State *state = as5600mux->getCurrentAS5600State();
    state->raw_angle = 360.0f/4096.0f * static_cast<float>(static_cast<uint16_t>(state->buffer[0] << 8 | state->buffer[1]));
    as5600mux->changeChannelDMA();
    as5600mux->updateDataDMA();
}

// static void mux_write_callback(void *ctx) {
//     const auto as5600mux = reinterpret_cast<Biped::AS5600::AS5600MUX *>(ctx);
//     as5600mux->updateDataDMA();
// }

namespace Biped::AS5600 {
    bool AS5600MUX::changeChannel() {
        this->active_as5600_index = (this->active_as5600_index + 1) % this->as5600_count;
        this->active_channel = this->as5600_states[this->active_as5600_index].mux_index;
        bool success = i2c::writeRegister(PCA9548A_ADDR, static_cast<uint8_t>(0b1 << this->active_channel), nullptr, 0,
                                          false);
        if (!success) {
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
        }
        return success;
    }

    AS5600State *AS5600MUX::getCurrentAS5600State() {
        return &as5600_states[this->active_as5600_index];
    }

    void AS5600MUX::changeChannelDMA() {
        this->active_as5600_index = (this->active_as5600_index + 1) % this->as5600_count;
        this->active_channel = this->as5600_states[this->active_as5600_index].mux_index;
        auto channel_mask = static_cast<uint8_t>(0b1 << this->active_channel);
        if (!i2c::writeRegister(PCA9548A_ADDR, channel_mask, nullptr, 0, false)) {
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
        }
    }

    void AS5600MUX::updateDataDMA() {
        // Loop until we successfully start a DMA read, to prevent infinite recursion
        // if multiple/all sensors are disconnected and we instantly NACK.
        for (int i = 0; i < this->as5600_count; i++) {
            bool success = i2c::readRegister(AS5600_ADDR, RAW_ANGLE_H, this->getCurrentAS5600State()->buffer, 2, true);
            if (success) {
                return; // DMA started successfully, callback will handle the rest
            }
            
            // Failed to start (NACK). Mark error, recover bus, and try the next channel.
            this->getCurrentAS5600State()->status = MagnetStatus::ReadError;
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
            this->changeChannelDMA();
        }
    }
    void AS5600MUX::start() {
        this->changeChannelDMA();
        this->updateDataDMA();
    }

    bool AS5600MUX::initialize() {
        i2c::enable(true);
        i2c::setCallbacks(encoder_read_callback, nullptr, this);
        for (uint8_t i = 0; i < this->as5600_count; i++) {
            changeChannel();
            readMagnetStatus();
        }
        return true;
    }

    void AS5600MUX::updateAngles() {
        unsigned int channel_mask = 0b1 << this->active_channel;
        AS5600State *state = &this->as5600_states[this->active_as5600_index];
        if (!i2c::writeRegister(PCA9548A_ADDR, channel_mask, nullptr, 0, false)) {
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
            state->status = MagnetStatus::ReadError;
            return;
        }

        if (!i2c::readRegister(AS5600_ADDR, RAW_ANGLE_H, state->buffer, 2)) {
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
            state->status = MagnetStatus::ReadError;
            return;
        }


        state->raw_angle = static_cast<float>(static_cast<uint16_t>(state->buffer[0] << 8 | state->buffer[1]))*360.0f / 4096.0f;
    }
    void AS5600MUX::readMagnetStatus() {
        unsigned int channel_mask = 0b1 << this->active_channel;
        AS5600State *state = &this->as5600_states[this->active_as5600_index];
        if (!i2c::writeRegister(PCA9548A_ADDR, channel_mask, nullptr, 0, false)) {
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
            state->status = MagnetStatus::ReadError;
            return;
        }

        uint8_t status = 0;
        if (!i2c::readRegister(AS5600_ADDR, STATUS, &status, 1)) {
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
            state->status = MagnetStatus::ReadError;
            return;
        }


        const bool md = status & (1 << 5);
        const bool ml = status & (1 << 4);
        const bool mh = status & (1 << 3);

        if (!md)
            state->status = MagnetStatus::NotDetected;
        if (ml)
            state->status = MagnetStatus::TooWeak;
        if (mh)
            state->status = MagnetStatus::TooStrong;
        state->status = MagnetStatus::OK;
    }
}
