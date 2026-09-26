//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_AS5600_H
#define BIPEDALV1_AS5600_H
#include <optional>
#include "drivers/I2C.h"
#include "drivers/Clock.h"

namespace Biped::MagneticEncoder {
    using i2c = F411::I2C1;
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

    struct Encoder {
        enum class EncoderType : uint8_t {
            MT6701,
            AS5600
        } type = EncoderType::MT6701;

        uint8_t mux_index{};
        std::optional<float> reference_angle{};
        float raw_angle{};
        float normalized_angle{};
        float rpm{};
        unsigned int last_read_time_us = 0;

        std::optional<MagnetStatus> status{MagnetStatus::NotDetected};
        uint8_t buffer[2] = {};

        void updateAngle();

        void readMagnetStatus();
    };

    // PCA9548A Multiplexer has been used
    class EncoderI2CMux {
        // ── Addresses ─────────────────────────────────────────────
        static inline uint8_t PCA9548A_ADDR = 0x70;
        static constexpr uint8_t AS5600_ADDR = 0x36;

        // ── AS5600 Register map ───────────────────────────────────


        unsigned int as5600_count = 4;
        Encoder encoder_states[4] = {
            {
                .type = Encoder::EncoderType::AS5600,
                .mux_index = 0, // hip right
            },
            {
                .type = Encoder::EncoderType::AS5600,
                .mux_index = 1, // hip left
            },
            {
                .type = Encoder::EncoderType::MT6701,
                .mux_index = 2, // wheel left
            },
            {
                .type = Encoder::EncoderType::MT6701,
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
        EncoderI2CMux() = default;

        bool initialize();

        void updateAngles();

        void readMagnetStatus();

        bool changeChannel();

        Encoder *getCurrentEncoder();

        bool changeChannelDMA();

        void updateDataDMA();

        void update();

        Encoder *getWheelLeft() { return &encoder_states[2]; };
        Encoder *getWheelRight() { return &encoder_states[3]; };
        Encoder *getHipLeft() { return &encoder_states[1]; };
        Encoder *getHipRight() { return &encoder_states[0]; };
    };
}
#endif //BIPEDALV1_AS5600_H
