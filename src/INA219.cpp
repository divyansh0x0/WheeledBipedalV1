//
// Created by divyansh on 9/5/26.
//

#include "INA219.h"

#include <bits/ranges_base.h>

#include "drivers/I2C.h"

namespace Biped {
    static void i2cReadCallback(void *ctx) {
        const auto ina219pair = static_cast<INA219Manager *>(ctx);
        ina219pair->is_ready = true;
    }

    void INA219Manager::initialize() {
        STM32F411::I2C3::enable();
        STM32F411::I2C3::setCallbacks(i2cReadCallback, nullptr, this);

        constexpr float shunt_resistance = 0.100f; // in ohms
        constexpr float max_current = 3; // in amps
        const float current_lsb = max_current / 32768;
        const auto calibration_register_val = static_cast<uint16_t>(
            0.04096f / (current_lsb * shunt_resistance));
        uint8_t data[2] = {};
        data[0] = (calibration_register_val >> 8) & 0xFF;
        data[1] = calibration_register_val & 0xFF;
        for (unsigned int i = 0; i < this->ina219_count; i++) {
            STM32F411::I2C3::writeRegister(this->ina219_data_arr[i].address,
                                           static_cast<uint8_t>(INA219MemoryMap::calibration),
                                           data, 2, false);
            ina219_data_arr[i].current_lsb = current_lsb;
        }
    }

    void INA219Manager::update() {
        using namespace STM32F411;
        using i2c = I2C3;

        if (!is_ready)
            return;
        if (last_read_time == 0) {
            last_read_time = Clock::micros();
            return;
        }
        const unsigned int curr_time = Clock::micros();
        if (curr_time - last_read_time < 400) {
            return;
        }
        last_read_time = curr_time;
        
        bool success = false;
        
        switch (current_state) {
            case INA219State::SHUNT_0:
                success = i2c::readRegister(ina219_data_arr[0].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::shunt_voltage),
                                            &ina219_data_arr[0].buffer[0], 2, true);
                current_state = INA219State::BUS_0;
                break;
                
            case INA219State::BUS_0:
                success = i2c::readRegister(ina219_data_arr[0].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::bus_voltage),
                                            &ina219_data_arr[0].buffer[2], 2, true);
                current_state = INA219State::POWER_0;
                break;

            case INA219State::POWER_0:
                success = i2c::readRegister(ina219_data_arr[0].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::power),
                                            &ina219_data_arr[0].buffer[4], 2, true);
                current_state = INA219State::CURRENT_0;
                break;
                
            case INA219State::CURRENT_0:
                success = i2c::readRegister(ina219_data_arr[0].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::current),
                                            &ina219_data_arr[0].buffer[6], 2, true);
                current_state = INA219State::SHUNT_1;
                break;
                
            case INA219State::SHUNT_1:
                success = i2c::readRegister(ina219_data_arr[1].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::shunt_voltage),
                                            &ina219_data_arr[1].buffer[0], 2, true);
                current_state = INA219State::BUS_1;
                break;

            case INA219State::BUS_1:
                success = i2c::readRegister(ina219_data_arr[1].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::bus_voltage),
                                            &ina219_data_arr[1].buffer[2], 2, true);
                current_state = INA219State::POWER_1;
                break;

            case INA219State::POWER_1:
                success = i2c::readRegister(ina219_data_arr[1].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::power),
                                            &ina219_data_arr[1].buffer[4], 2, true);
                current_state = INA219State::CURRENT_1;
                break;
                
            case INA219State::CURRENT_1:
                success = i2c::readRegister(ina219_data_arr[1].address, 
                                            static_cast<uint8_t>(INA219MemoryMap::current),
                                            &ina219_data_arr[1].buffer[6], 2, true);
                current_state = INA219State::SHUNT_0;
                break;
        }

        if (success) {
            is_ready = false;
        }
    }
}
