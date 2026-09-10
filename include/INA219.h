//
// Created by divyansh on 9/5/26.
//

#ifndef BIPEDALV1_INA219_H
#define BIPEDALV1_INA219_H
#include <cstdint>

namespace Biped {
    enum class INA219MemoryMap : uint16_t {
        configuration = 0x0,
        shunt_voltage = 0x1,
        bus_voltage = 0x2,
        power = 0x3,
        current = 0x4,
        calibration = 0x5,
    };

    struct INA219Data {
        uint8_t address;
        uint8_t buffer[4 * 2] = {};
        float current_lsb = 0;

        [[nodiscard]] float getVoltage() const {
            // Combine MSB and LSB into a 16-bit unsigned integer
            uint16_t bus_voltage_raw = (buffer[2] << 8) | buffer[3];

            // Shift right by 3 to remove the status flags
            uint16_t bus_voltage_shifted = bus_voltage_raw >> 3;

            // Multiply by 4mV (0.004f) step size
            return static_cast<float>(bus_voltage_shifted) * 0.004f;
        }
        [[nodiscard]] float getPower() const {
            return static_cast<float>(static_cast<int16_t>((buffer[4] << 8u) | buffer[5])) * current_lsb * 20;
        }
        [[nodiscard]] float getCurrent() const {
            return static_cast<float>(static_cast<int16_t>((buffer[6] << 8u) | buffer[7])) * current_lsb;
        }


    };

    enum class INA219State {
        SHUNT_0,
        BUS_0,
        POWER_0,
        CURRENT_0,
    };

    class INA219Manager {

        unsigned int last_read_time = 0;
        unsigned int current_ina219_index = 0;

    public:
        INA219State current_state = INA219State::SHUNT_0;

        INA219Data ina219_data_arr[2] = {
            {.address = 0x40},
            {.address = 0x41},
        };
        unsigned int ina219_count = 2;
        bool is_ready = true;

        void initialize();


        void incrementIndex() {
            this->current_ina219_index =  (this->current_ina219_index + 1) % ina219_count ;
        }

        void update();
    };
}
#endif //BIPEDALV1_INA219_H
