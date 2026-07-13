//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_AS5600_H
#define BIPEDALV1_AS5600_H
#include <concepts>
#include "MemoryMap.h"
#include "I2C.h"

namespace STM32F411::AS5600 {

    template<typename T>
    concept I2CType = std::same_as<T, I2C1> ||
                      std::same_as<T, I2C2> ||
                      std::same_as<T, I2C3>;

    /**
     * Magnet status reported by the AS5600 STATUS register (0x0B).
     *   MD (bit 5) – magnet detected
     *   ML (bit 4) – magnet too weak  (AGC at max)
     *   MH (bit 3) – magnet too strong (AGC at min)
     */
    enum class MagnetStatus : uint8_t {
        OK          = 0,   // Magnet detected, field strength in range
        NotDetected = 1,   // No magnet sensed at all
        TooWeak     = 2,   // Magnet present but field is too weak
        TooStrong   = 3,   // Magnet present but field is too strong
        ReadError   = 4,   // I2C / mux communication failure
    };

    /**
     * Driver for one or more AS5600 magnetic rotary encoders connected
     * behind a PCA9548A I2C multiplexer.
     *
     * Usage (mirrors MPU6050 pattern):
     *   AS5600::AS5600MUX<I2C2> encoder;
     *   encoder.configure(2);     // mux channel 2
     *   auto status = encoder.readMagnetStatus();
     *   encoder.readAngle(angle);
     *
     * @tparam i2c  The I2C peripheral type (I2C1, I2C2, I2C3)
     */
    template<I2CType i2c>
    class AS5600MUX {
        // ── Addresses ─────────────────────────────────────────────
        static constexpr uint8_t PCA9548A_ADDR = 0x70;   // A0=A1=A2=GND
        static constexpr uint8_t AS5600_ADDR   = 0x36;

        // ── AS5600 Register map ───────────────────────────────────
        enum Registers : uint8_t {
            STATUS      = 0x0B,
            RAW_ANGLE_H = 0x0C,   // 12-bit raw angle [11:8]
            RAW_ANGLE_L = 0x0D,   // 12-bit raw angle  [7:0]
            ANGLE_H     = 0x0E,   // 12-bit filtered angle [11:8]
            ANGLE_L     = 0x0F,   // 12-bit filtered angle  [7:0]
            AGC         = 0x1A,
            MAGNITUDE_H = 0x1B,
            MAGNITUDE_L = 0x1C,
        };

        // ── Instance data ─────────────────────────────────────────
        uint8_t channel = 0;
        uint8_t buffer[2] = {};

        /**
         * Select this encoder's channel on the PCA9548A.
         * The PCA9548A has no register pointer — a single byte
         * written after the address sets the channel mask.
         * Uses writeByte: START → 0x70+W → mask → STOP
         */
        bool selectChannel() {
            return i2c::writeByte(PCA9548A_ADDR, static_cast<uint8_t>(1u << channel));
        }

    public:
        AS5600MUX() = default;

        /**
         * Initialise the I2C peripheral and store the mux channel.
         * Call once before any reads. Mirrors MPU6050::configure().
         *
         * @param mux_channel  PCA9548A channel this encoder is on (0-7)
         */
        void configure(uint8_t mux_channel) {
            channel = mux_channel;
            i2c::enable();
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
            if (!selectChannel()) return MagnetStatus::ReadError;

            uint8_t status = 0;
            if (!i2c::readRegister(AS5600_ADDR, Registers::STATUS, &status, 1))
                return MagnetStatus::ReadError;

            const bool md = status & (1 << 5);
            const bool ml = status & (1 << 4);
            const bool mh = status & (1 << 3);

            if (!md) return MagnetStatus::NotDetected;
            if (ml)  return MagnetStatus::TooWeak;
            if (mh)  return MagnetStatus::TooStrong;
            return MagnetStatus::OK;
        }

        /**
         * Read the raw 12-bit angle and convert to degrees.
         *
         * @param angle_out  Receives the angle in degrees (0.0 – 360.0)
         * @return true on success
         */
        bool readAngle(volatile float &angle_out) {
            if (!selectChannel()) return false;

            if (!i2c::readRegister(AS5600_ADDR, Registers::RAW_ANGLE_H, buffer, 2))
                return false;

            const uint16_t raw = (static_cast<uint16_t>(buffer[0] & 0x0F) << 8) | buffer[1];
            angle_out = (static_cast<float>(raw) / 4096.0f) * 360.0f;
            return true;
        }

        /**
         * Read the 12-bit raw angle value without degree conversion.
         *
         * @param raw_out  Receives the raw 12-bit value (0 – 4095)
         * @return true on success
         */
        bool readRawAngle(volatile uint16_t &raw_out) {
            if (!selectChannel()) return false;

            if (!i2c::readRegister(AS5600_ADDR, Registers::RAW_ANGLE_H, buffer, 2))
                return false;

            raw_out = (static_cast<uint16_t>(buffer[0] & 0x0F) << 8) | buffer[1];
            return true;
        }

        /**
         * Convenience: check whether a magnet is detected at all.
         */
        [[nodiscard]] bool isMagnetDetected() {
            const auto s = readMagnetStatus();
            return s != MagnetStatus::NotDetected && s != MagnetStatus::ReadError;
        }
    };
}
#endif //BIPEDALV1_AS5600_H
