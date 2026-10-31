#ifndef BIPEDALV1_AS5600_H
#define BIPEDALV1_AS5600_H
#include <optional>
#include "drivers/I2C.h"
#include "drivers/Clock.h"

namespace Biped::MagneticEncoder {
    using i2c = F411::I2C1;

    enum class MagnetStatus : uint8_t {
        OK = 0,
        NotDetected = 1,
        TooWeak = 2,
        TooStrong = 3,
        ReadError = 4,
    };

    static constexpr size_t VELOCITY_HISTORY_SIZE = 16;
    static constexpr float DEFAULT_RPM_EMA_ALPHA = 0.25f;

    class LowPassFilterEMA {
        float m_alpha = 1.0f;
        float m_filtered_value = 0.0f;
        bool m_is_initialized = false;

    public:
        constexpr LowPassFilterEMA() = default;
        constexpr explicit LowPassFilterEMA(float alpha) : m_alpha(alpha) {}

        void setAlpha(float alpha) {
            m_alpha = alpha;
        }

        [[nodiscard]] float getAlpha() const {
            return m_alpha;
        }

        void reset(float initial_value = 0.0f) {
            m_filtered_value = initial_value;
            m_is_initialized = true;
        }

        float update(float input) {
            if (!m_is_initialized) {
                m_filtered_value = input;
                m_is_initialized = true;
                return m_filtered_value;
            }
            m_filtered_value = (m_alpha * input) + ((1.0f - m_alpha) * m_filtered_value);
            return m_filtered_value;
        }

        [[nodiscard]] float getValue() const {
            return m_filtered_value;
        }
    };

    struct Encoder {
        enum class EncoderType : uint8_t {
            MT6701,
            AS5600
        } type = EncoderType::MT6701;

        uint8_t mux_index{};
        std::optional<float> reference_angle{};
        float raw_angle{};
        float normalized_angle{};
        float raw_rpm{};
        float rpm{};
        float last_differentiate_angle{};
        unsigned int last_read_time_us = 0;
        unsigned int last_differentiate_time_us = 0;
        bool is_inverted = false;
        LowPassFilterEMA rpm_filter{DEFAULT_RPM_EMA_ALPHA};

        std::optional<MagnetStatus> status{MagnetStatus::NotDetected};
        uint8_t buffer[2] = {};

        void updateAngle();

        void differentiate();

        void readMagnetStatus();
    };

    class EncoderI2CMux {
        static inline uint8_t PCA9548A_ADDR = 0x70;
        static constexpr uint8_t AS5600_ADDR = 0x36;

        unsigned int as5600_count = 4;
        Encoder encoder_states[4] = {
            {
                .type = Encoder::EncoderType::AS5600,
                .mux_index = 0,
            },
            {
                .type = Encoder::EncoderType::AS5600,
                .mux_index = 1,
            },
            {
                .type = Encoder::EncoderType::MT6701,
                .mux_index = 2,
            },
            {
                .type = Encoder::EncoderType::MT6701,
                .mux_index = 5,
                .is_inverted = true,
            }
        };
        uint8_t active_as5600_index = 0;
        uint8_t active_channel = 0;
        unsigned int current_mux_index = 0;

    public:
        uint8_t detected_channel_addresses[8] = {};

        EncoderI2CMux() = default;

        bool initialize();

        void updateAngles();

        void readMagnetStatus();

        bool changeChannel();

        Encoder *getCurrentEncoder();

        bool changeChannelDMA();

        void updateDataDMA();

        void update();

        void differentiate();

        Encoder *getWheelLeft() { return &encoder_states[2]; };
        Encoder *getWheelRight() { return &encoder_states[3]; };
        Encoder *getHipLeft() { return &encoder_states[1]; };
        Encoder *getHipRight() { return &encoder_states[0]; };
    };
}
#endif //BIPEDALV1_AS5600_H
