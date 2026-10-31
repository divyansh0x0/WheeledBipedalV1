#ifndef BIPEDALV1_PERIPHERALS_H
#define BIPEDALV1_PERIPHERALS_H
#include <type_traits>
#include "drivers/MemoryMap.h"

namespace F411 {
    template<typename Peripheral, typename Pin>
    struct PeripheralTraits;

    template<typename Peripheral, typename Pin, typename = void>
    struct is_valid_mapping : std::false_type {
    };

    template<typename Peripheral, typename Pin>
    struct is_valid_mapping<Peripheral, Pin, std::void_t<decltype(PeripheralTraits<Peripheral, Pin>::af
            )> > : std::true_type {
    };

    namespace Peripherals {
        struct TIMER2 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::PushPull;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::None;
        };
        struct TIMER3 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::PushPull;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::None;
        };
        struct TIMER4 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::PushPull;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::None;
        };

        struct TIMER5 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::PushPull;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::None;
        };

        struct SCL1 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::Up;
        };

        struct SDA1 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::Up;
        };
        struct SCL3 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::Up;
        };

        struct SDA3 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::Up;
        };

        struct SCL2 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::Up;
        };

        struct SDA2 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::Up;
        };
    }
}
#endif //BIPEDALV1_PERIPHERALS_H
