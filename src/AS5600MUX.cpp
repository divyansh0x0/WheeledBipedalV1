//
// Created by divyansh on 9/10/26.
//

#include "AS5600MUX.h"

static void read_callback(void *ctx) {
    const auto as5600mux = reinterpret_cast<Biped::AS5600::AS5600MUX *>(ctx);
    if (!as5600mux->change_channel()) {
        as5600mux->getCurrentAS5600State();
    }
}
namespace Biped::AS5600 {
    bool AS5600MUX::change_channel() {
        this->active_as5600_index = this->active_as5600_index % this->as5600_count + 1;
        this->active_channel = this->as5600_states[this->active_as5600_index].mux_index;
        return i2c::writeByte(PCA9548A_ADDR, static_cast<uint8_t>(0b1 << this->active_channel));
    }

    AS5600State *AS5600MUX::getCurrentAS5600State() {
        return &as5600_states[this->active_as5600_index];
    }

    bool AS5600MUX::initialize() {
        i2c::enable();
        i2c::setCallbacks(read_callback, nullptr, this);
        // for (uint8_t i = 0; i < as5600_count; i++) {
        //     const uint8_t channel = this->as5600_states[this->active_as5600_index].mux_index;
        //     i2c::writeByte(PCA9548A_ADDR, channel);
        //     const auto magnet_status = this->readMagnetStatus();
        //     if (magnet_status == MagnetStatus::OK) {
        //         continue;
        //     }
        //     else {
        //         return false;
        //     }
        // }
        const uint8_t channel = 7;
        i2c::writeByte(PCA9548A_ADDR, channel);
        return true;
    }

    MagnetStatus AS5600MUX::readMagnetStatus() {
        uint8_t status = 0;
        if (!i2c::readRegister(AS5600_ADDR, STATUS, &status, 1))
            return MagnetStatus::ReadError;


        const bool md = status & (1 << 5);
        const bool ml = status & (1 << 4);
        const bool mh = status & (1 << 3);

        if (!md) return MagnetStatus::NotDetected;
        if (ml) return MagnetStatus::TooWeak;
        if (mh) return MagnetStatus::TooStrong;
        return MagnetStatus::OK;
    }

    bool AS5600MUX::readRawAngle(volatile uint16_t &raw_out) {
        if (!i2c::readRegister(AS5600_ADDR, RAW_ANGLE_H, &buffer[current_mux_index * 2], 2))
            return false;

        raw_out = (static_cast<uint16_t>(buffer[0] & 0x0F) << 8) | buffer[1];
        return true;
    }

    bool AS5600MUX::isMagnetDetected() {
        const auto s = readMagnetStatus();
        return s != MagnetStatus::NotDetected && s != MagnetStatus::ReadError;
    }
}
