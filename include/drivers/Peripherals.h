//
// Created by divyansh on 7/2/26.
//

#ifndef BIPEDALV1_PERIPHERALS_H
#define BIPEDALV1_PERIPHERALS_H
#include <type_traits>
#include "drivers/MemoryMap.h"


namespace STM32F411 {
    template<typename Peripheral, typename Pin>
    struct PeripheralTraits;

    // 1. Base case: The trait does not exist (invalid pin mapping)
    template<typename Peripheral, typename Pin, typename = void>
    struct is_valid_mapping : std::false_type {
    };

    // 2. Specialization: The trait exists (valid pin mapping)
    template<typename Peripheral, typename Pin>
    struct is_valid_mapping<Peripheral, Pin, std::void_t<decltype(PeripheralTraits<Peripheral, Pin>::af
            )> > : std::true_type {
    };

    namespace Peripherals {
        struct SCL1 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::None;
        };

        struct SDA1 {
            static constexpr auto type = MemoryMap::GPIORegister::OutputType::OpenDrain;
            static constexpr auto speed = MemoryMap::GPIORegister::OutputSpeed::High_100MHz;
            static constexpr auto pull = MemoryMap::GPIORegister::Pull::None;
        };
    }
}
#endif //BIPEDALV1_PERIPHERALS_H
