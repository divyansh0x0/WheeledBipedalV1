//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_AS5600_H
#define BIPEDALV1_AS5600_H
#include <concepts>
#include "drivers/MemoryMap.h"
#include "drivers/I2C.h"
#include "drivers/Clock.h"

namespace STM32F411::AS5600 {
    /**
     * Magnet status reported by the AS5600 STATUS register (0x0B).
     *   MD (bit 5) – magnet detected
     *   ML (bit 4) – magnet too weak  (AGC at max)
     *   MH (bit 3) – magnet too strong (AGC at min)
     */
    enum class MagnetStatus : uint8_t {
        OK = 0, // Magnet detected, field strength in range
        NotDetected = 1, // No magnet sensed at all
        TooWeak = 2, // Magnet present but field is too weak
        TooStrong = 3, // Magnet present but field is too strong
        ReadError = 4, // I2C / mux communication failure
    };

    struct AS5600State {
        uint8_t mux_index;
        float raw_angle;
        float normalized_angle;
        float rpm;
        MagnetStatus status;
    };

    class AS5600MUX {
        using i2c = I2C1;

        // ── Addresses ─────────────────────────────────────────────
        static constexpr uint8_t PCA9548A_ADDR = 0x70; // A0=A1=A2=GND
        static constexpr uint8_t AS5600_ADDR = 0x36;

        // ── AS5600 Register map ───────────────────────────────────
        enum Registers : uint8_t {
            STATUS = 0x0B,
            RAW_ANGLE_H = 0x0C, // 12-bit raw angle [11:8]
            RAW_ANGLE_L = 0x0D, // 12-bit raw angle  [7:0]
            ANGLE_H = 0x0E, // 12-bit filtered angle [11:8]
            ANGLE_L = 0x0F, // 12-bit filtered angle  [7:0]
            AGC = 0x1A,
            MAGNITUDE_H = 0x1B,
            MAGNITUDE_L = 0x1C,
        };

        unsigned int as5600_count = 4;
        AS5600State as5600_states[4] = {
            {
                .mux_index = 0, .raw_angle = 0.0f, .normalized_angle = 0.0f, .rpm = 0.0f,
                .status = MagnetStatus::NotDetected
            },
            {
                .mux_index = 1, .raw_angle = 0.0f, .normalized_angle = 0.0f, .rpm = 0.0f,
                .status = MagnetStatus::NotDetected
            },
            {
                .mux_index = 2, .raw_angle = 0.0f, .normalized_angle = 0.0f, .rpm = 0.0f,
                .status = MagnetStatus::NotDetected
            },
            {
                .mux_index = 3, .raw_angle = 0.0f, .normalized_angle = 0.0f, .rpm = 0.0f,
                .status = MagnetStatus::NotDetected
            }
        };
        uint8_t active_as5600_index = 0;
        uint8_t active_channel = 0;
        uint8_t buffer[2 * 4] = {};
        /**
         * Select this encoder's active_channel on the PCA9548A.
         * The PCA9548A has no register pointer — a single byte
         * written after the address sets the active_channel mask.
         * Uses writeByte: START → 0x70+W → mask → STOP
         */
        inline bool update_channel() {
            this->active_as5600_index = this->active_as5600_index % 4 + 1;
            return i2c::writeByte(PCA9548A_ADDR, static_cast<uint8_t>(1u << active_channel));
        }

    public:
        AS5600MUX() = default;

        unsigned int current_mux_index;

        AS5600State* getCurrentAS5600State() {
        }

        void configure() {
            i2c::enable();
            const I2CReadCallback read_callback = [](void *ctx) {
                const auto as5600mux = reinterpret_cast<AS5600MUX *>(ctx);
                if (!as5600mux->update_channel()) {
                    as5600mux->getCurrentAS5600State();
                }
            };
            i2c::setCallbacks(read_callback, nullptr, this);
        }

        /**
         * Read the full magnet status from the AS5600.
         *
         * STATUS register (0x0B) bits:
         *   Bit 5 (MD) – Magnet detected
         *   Bit 4 (ML) – Magnet too weak
         *   Bit 3 (MH) – Magnet too strong
         */
        MagnetStatus readMagnetStatus() {

            uint8_t status = 0;
            if (!i2c::readRegister(AS5600_ADDR, Registers::STATUS, &status, 1))
                return MagnetStatus::ReadError;


            const bool md = status & (1 << 5);
            const bool ml = status & (1 << 4);
            const bool mh = status & (1 << 3);

            if (!md) return MagnetStatus::NotDetected;
            if (ml) return MagnetStatus::TooWeak;
            if (mh) return MagnetStatus::TooStrong;
            return MagnetStatus::OK;
        }

        bool readRawAngle(volatile uint16_t &raw_out) {
            if (!i2c::readRegister(AS5600_ADDR, RAW_ANGLE_H, &buffer[current_mux_index * 2], 2))
                return false;

            raw_out = (static_cast<uint16_t>(buffer[0] & 0x0F) << 8) | buffer[1];
            return true;
        }

        [[nodiscard]] bool isMagnetDetected() {
            const auto s = readMagnetStatus();
            return s != MagnetStatus::NotDetected && s != MagnetStatus::ReadError;
        }
    };
}
#endif //BIPEDALV1_AS5600_H
