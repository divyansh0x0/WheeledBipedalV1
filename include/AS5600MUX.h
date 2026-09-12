//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_AS5600_H
#define BIPEDALV1_AS5600_H
#include <concepts>
#include <optional>

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

    static constexpr size_t VELOCITY_HISTORY_SIZE = 16;

    struct AS5600State {
        uint8_t mux_index{};
        float raw_angle{};
        std::optional<float>  reference_angle{};
        float normalized_angle{};
        float rpm{};
        unsigned int last_read_time_us = 0;
        
        // Velocity estimation variables
        float continuous_angle{};
        float angle_history[VELOCITY_HISTORY_SIZE]{};
        unsigned int time_history_us[VELOCITY_HISTORY_SIZE]{};
        size_t history_index{};
        bool history_filled{};

        MagnetStatus status{MagnetStatus::NotDetected};
        uint8_t buffer[2] = {};
    };

    // PCA9548A Multiplexer has been used
    class AS5600MUX {
        using i2c = I2C1;

        // ── Addresses ─────────────────────────────────────────────
        static inline uint8_t PCA9548A_ADDR = 0x70;
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

        unsigned int as5600_count = 4;
        AS5600State as5600_states[4] = {
            {
                .mux_index = 0, // hip right
            },
            {
                .mux_index = 1, // hip left
            },
            {
                .mux_index = 2, // wheel left
            },
            {
                .mux_index = 3, //wheel right
            }
        };
        uint8_t active_as5600_index = 0;
        uint8_t active_channel = 0;
        /**
         * Select this encoder's active_channel on the PCA9548A.
         * The PCA9548A has no register pointer — a single byte
         * written after the address sets the active_channel mask.
         * Uses writeByte: START → 0x70+W → mask → STOP
         */

        unsigned int current_mux_index = 0;

    public:
        AS5600MUX() = default;

        bool initialize();

        void updateAngles();

        void readMagnetStatus();

        bool changeChannel();

        AS5600State *getCurrentAS5600State();

        bool changeChannelDMA();

        void updateDataDMA();

        void update();

        AS5600State * getWheelLeft() {return &as5600_states[2];};
        AS5600State * getWheelRight(){return &as5600_states[3];};
        AS5600State * getHipLeft(){return &as5600_states[1];};
        AS5600State * getHipRight(){return &as5600_states[0];};
    };
}
#endif //BIPEDALV1_AS5600_H
