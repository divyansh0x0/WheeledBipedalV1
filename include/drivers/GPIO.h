#ifndef WHEEL2FIRMWARE_GPIO_H
#define WHEEL2FIRMWARE_GPIO_H
#include "GPIO/Peripherals.h"
#include "drivers/MemoryMap.h"


namespace STM32F411 {
    namespace AF {

    }
    template<unsigned int addr, unsigned int index>
    class GPIO {
    public:
        // Undefined - will be set manually later
        enum class AlternateFunction;

        enum Status {
            HIGH, LOW,
        };


        static void toggle() {
            const auto port = reinterpret_cast<MemoryMap::GPIORegister *>(addr);
            if (port->ODR & (1 << index)) // check if pin was suppose to be high
                port->setPinLow(index);
            else
                port->setPinHigh(index);
        }

        static void set(Status status) {
            const auto port = reinterpret_cast<MemoryMap::GPIORegister *>(addr);
            if (status == HIGH) {
                port->setPinHigh(index);
            } else {
                port->setPinLow(index);
            }
        }
        static void enableAnalogMode() {
            const auto port = reinterpret_cast<MemoryMap::GPIORegister *>(addr);
            port->configurePin(index, MemoryMap::GPIORegister::Mode::Analog);
        }
        static Status getStatus() {
            const auto port = reinterpret_cast<MemoryMap::GPIORegister *>(addr);
            return port->ODR & (1 << index) ? HIGH : LOW;
        }
        static void enableOutputMode() {
            const auto port = reinterpret_cast<MemoryMap::GPIORegister *>(addr);
            port->configurePin(index, MemoryMap::GPIORegister::Mode::Output);
        }
        template<typename Peripheral>
        static void alternateFunction() {
            static_assert(is_valid_mapping<Peripheral, GPIO>::value,
                          "HARDWARE ERROR: This peripheral cannot be routed to this specific GPIO pin!");
            constexpr uint8_t af = PeripheralTraits<Peripheral, GPIO>::af;
            const auto port = reinterpret_cast<MemoryMap::GPIORegister *>(addr);
            constexpr uint8_t reg = index/8;
            constexpr uint8_t bitPos = (index % 8) * 4; // multiply by 4 because
            port->AFR[reg] &= ~(0b1111 << bitPos);
            port->AFR[reg] |= static_cast<uint8_t>(af) << bitPos;

            port->configurePin(index, MemoryMap::GPIORegister::Mode::AlternateFunction);
        }
    };

    namespace Pins {
        using A0 = GPIO<MemoryMap::GPIOAdressA, 0>;
        using A1 = GPIO<MemoryMap::GPIOAdressA, 1>;
        using A2 = GPIO<MemoryMap::GPIOAdressA, 2>;
        using A3 = GPIO<MemoryMap::GPIOAdressA, 3>;
        using A4 = GPIO<MemoryMap::GPIOAdressA, 4>;
        using A5 = GPIO<MemoryMap::GPIOAdressA, 5>;
        using A6 = GPIO<MemoryMap::GPIOAdressA, 6>;
        using A7 = GPIO<MemoryMap::GPIOAdressA, 7>;
        using A8 = GPIO<MemoryMap::GPIOAdressA, 8>;
        using A9 = GPIO<MemoryMap::GPIOAdressA, 9>;
        using A10 = GPIO<MemoryMap::GPIOAdressA, 10>;
        using A11 = GPIO<MemoryMap::GPIOAdressA, 11>;
        using A12 = GPIO<MemoryMap::GPIOAdressA, 12>;
        using A13 = GPIO<MemoryMap::GPIOAdressA, 13>;
        using A14 = GPIO<MemoryMap::GPIOAdressA, 14>;
        using A15 = GPIO<MemoryMap::GPIOAdressA, 15>;

        using B0 = GPIO<MemoryMap::GPIOAdressB, 0>;
        using B1 = GPIO<MemoryMap::GPIOAdressB, 1>;
        using B2 = GPIO<MemoryMap::GPIOAdressB, 2>;
        using B3 = GPIO<MemoryMap::GPIOAdressB, 3>;
        using B4 = GPIO<MemoryMap::GPIOAdressB, 4>;
        using B5 = GPIO<MemoryMap::GPIOAdressB, 5>;
        using B6 = GPIO<MemoryMap::GPIOAdressB, 6>;
        using B7 = GPIO<MemoryMap::GPIOAdressB, 7>;
        using B8 = GPIO<MemoryMap::GPIOAdressB, 8>;
        using B9 = GPIO<MemoryMap::GPIOAdressB, 9>;
        using B10 = GPIO<MemoryMap::GPIOAdressB, 10>;
        using B11 = GPIO<MemoryMap::GPIOAdressB, 11>;
        using B12 = GPIO<MemoryMap::GPIOAdressB, 12>;
        using B13 = GPIO<MemoryMap::GPIOAdressB, 13>;
        using B14 = GPIO<MemoryMap::GPIOAdressB, 14>;
        using B15 = GPIO<MemoryMap::GPIOAdressB, 15>;

        using C13 = GPIO<MemoryMap::GPIOAdressC, 13>;
        using C14 = GPIO<MemoryMap::GPIOAdressC, 14>;
        using C15 = GPIO<MemoryMap::GPIOAdressC, 15>;
    }



    template<> struct PeripheralTraits<Peripherals::SCL1, Pins::B8> { static constexpr uint8_t af = 4; };
    template<> struct PeripheralTraits<Peripherals::SCL1, Pins::B6> { static constexpr uint8_t af = 4; };

    template<> struct PeripheralTraits<Peripherals::SDA1, Pins::B7> { static constexpr uint8_t af = 4; };
    template<> struct PeripheralTraits<Peripherals::SDA1,  Pins::B9> { static constexpr uint8_t af = 4; };




}


#endif
