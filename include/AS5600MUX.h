//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_AS5600_H
#define BIPEDALV1_AS5600_H
#include <concepts>
#include "drivers/MemoryMap.h"
#include "drivers/I2C.h"
#include "drivers/Clock.h"

namespace Biped::AS5600 {
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
        static constexpr uint8_t PCA9548A_ADDR = 0x70;
        static constexpr uint8_t AS5600_ADDR = 0x36;

        // ── AS5600 Register map ───────────────────────────────────
        enum Registers : uint8_t {
            ZMCO = 0x0,
            ZPOS_L = 0x01,
            ZPOS_H = 0x02,
            MPOS_L = 0x03,
            MPOS_H = 0x04,
            MANG_L = 0x05,
            MANG_H = 0x06,
            CONF_L = 0x07,
            CONF_H = 0x08,
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

        unsigned int current_mux_index;

    public:
        AS5600MUX() = default;

        bool initialize();

        MagnetStatus readMagnetStatus();

        bool readRawAngle(volatile uint16_t &raw_out);

        bool change_channel();

        [[nodiscard]] bool isMagnetDetected();

        AS5600State *getCurrentAS5600State();
    };
}
#endif //BIPEDALV1_AS5600_H
