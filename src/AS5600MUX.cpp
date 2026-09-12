//
// Created by divyansh on 9/10/26.
//

#include "AS5600MUX.h"
#include "drivers/GPIO.h"

static float normalize_angle(float angle) {
    return angle > 180 ? angle - 360 : angle < -180 ? angle + 360 : angle;
}

static void encoder_read_callback(void *ctx) {
    const auto as5600mux = reinterpret_cast<Biped::AS5600::AS5600MUX *>(ctx);
    Biped::AS5600::AS5600State *state = as5600mux->getCurrentAS5600State();
    const float new_angle =
            static_cast<float>(static_cast<uint16_t>(state->buffer[0] << 8 | state->buffer[1])) *
            360.0f
            / 4096.0f;


    const unsigned int current_time = Biped::Clock::micros();
    const float angle_change = normalize_angle(new_angle - state->raw_angle);
    
    // Accumulate continuous angle to prevent wrap-around across the history window
    state->continuous_angle += angle_change;
    state->raw_angle = new_angle;

    // Insert current point into history
    state->angle_history[state->history_index] = state->continuous_angle;
    state->time_history_us[state->history_index] = current_time;
    
    state->history_index++;
    if (state->history_index >= Biped::AS5600::VELOCITY_HISTORY_SIZE) {
        state->history_index = 0;
        state->history_filled = true;
    }

    float instant_rpm = 0.0f;
    if (state->history_filled) {
        // history_index points to the oldest entry because of wrap-around
        const float old_angle = state->angle_history[state->history_index];
        const unsigned int old_time = state->time_history_us[state->history_index];
        const unsigned int dt = current_time - old_time;
        if (dt > 0) {
            const float angle_diff = state->continuous_angle - old_angle;
            const float angular_velocity = angle_diff / static_cast<float>(dt);
            instant_rpm = angular_velocity / 360.0f * 1e6f * 60.0f;
        }
    } else if (state->history_index > 1) { // Fallback while buffer is filling
        const float old_angle = state->angle_history[0];
        const unsigned int old_time = state->time_history_us[0];
        const unsigned int dt = current_time - old_time;
        if (dt > 0) {
            const float angle_diff = state->continuous_angle - old_angle;
            const float angular_velocity = angle_diff / static_cast<float>(dt);
            instant_rpm = angular_velocity / 360.0f * 1e6f * 60.0f;
        }
    }

    // Heavy smoothing is required to handle the noise when the AS5600 Fast Filter engages
    const float filtered_rpm = (0.95f * state->rpm) + (0.05f * instant_rpm);
    state->rpm = (filtered_rpm) * filtered_rpm < 0.001f ? 0.0f : filtered_rpm;
    state->normalized_angle = normalize_angle(state->raw_angle - state->reference_angle.value());
    state->last_read_time_us = current_time;
    if (as5600mux->changeChannelDMA())
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

    bool AS5600MUX::changeChannelDMA() {
        this->active_as5600_index = (this->active_as5600_index + 1) % this->as5600_count;
        this->active_channel = this->as5600_states[this->active_as5600_index].mux_index;
        auto channel_mask = static_cast<uint8_t>(0b1 << this->active_channel);
        if (!i2c::writeRegister(PCA9548A_ADDR, channel_mask, nullptr, 0, false)) {
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
        }
        return this->active_channel != 0;
    }

    void AS5600MUX::updateDataDMA() {
        // Loop until we successfully update a DMA read, to prevent infinite recursion
        // if multiple/all sensors are disconnected and we instantly NACK.
        for (int i = 0; i < this->as5600_count; i++) {
            bool success = i2c::readRegister(AS5600_ADDR, RAW_ANGLE_H, this->getCurrentAS5600State()->buffer, 2, true);
            if (success) {
                return; // DMA started successfully, callback will handle the rest
            }

            // Failed to update (NACK). Mark error, recover bus, and try the next channel.
            this->getCurrentAS5600State()->status = MagnetStatus::ReadError;
            i2c::recoverBus<Pins::B6, Pins::B7, Peripherals::SCL1, Peripherals::SDA1>();
            if (!this->changeChannelDMA()) {
                return; // Reached channel 0 (end of cascade), stop!
            }
        }
    }

    void AS5600MUX::update() {
        this->updateDataDMA();
    }

    bool AS5600MUX::initialize() {
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


        const float new_angle = normalize_angle(
            static_cast<float>(static_cast<uint16_t>(state->buffer[0] << 8 | state->buffer[1])) *
            360.0f
            / 4096.0f);


        const unsigned int current_time = Clock::micros();
        if (state->last_read_time_us == 0) {
            state->last_read_time_us = current_time;
            state->raw_angle = new_angle;
            state->continuous_angle = new_angle;
            state->history_index = 0;
            state->history_filled = false;
        }

        const float angle_change = normalize_angle(new_angle - state->raw_angle);
        state->continuous_angle += angle_change;
        state->raw_angle = new_angle;

        state->angle_history[state->history_index] = state->continuous_angle;
        state->time_history_us[state->history_index] = current_time;
        
        state->history_index++;
        if (state->history_index >= VELOCITY_HISTORY_SIZE) {
            state->history_index = 0;
            state->history_filled = true;
        }

        float instant_rpm = 0.0f;
        if (state->history_filled) {
            const float old_angle = state->angle_history[state->history_index];
            const unsigned int old_time = state->time_history_us[state->history_index];
            const unsigned int dt = current_time - old_time;
            if (dt > 0) {
                const float angle_diff = state->continuous_angle - old_angle;
                const float angular_velocity = angle_diff / static_cast<float>(dt);
                instant_rpm = angular_velocity / 360.0f * 1e6f * 60.0f;
            }
        } else if (state->history_index > 1) {
            const float old_angle = state->angle_history[0];
            const unsigned int old_time = state->time_history_us[0];
            const unsigned int dt = current_time - old_time;
            if (dt > 0) {
                const float angle_diff = state->continuous_angle - old_angle;
                const float angular_velocity = angle_diff / static_cast<float>(dt);
                instant_rpm = angular_velocity / 360.0f * 1e6f * 60.0f;
            }
        }

        // Heavy smoothing is required to handle the noise when the AS5600 Fast Filter engages
        const float filtered_rpm = (0.95f * state->rpm) + (0.05f * instant_rpm);
        state->rpm = (filtered_rpm) * filtered_rpm < 0.001f ? 0.0f : filtered_rpm;

        if (!state->reference_angle.has_value()) {
            state->reference_angle = state->raw_angle;
        }
        state->normalized_angle = normalize_angle(state->raw_angle - state->reference_angle.value());
        state->last_read_time_us = current_time;
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
