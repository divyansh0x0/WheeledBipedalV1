//
// Created by divyansh on 7/8/26.
//

#ifndef BIPEDALV1_INTERRUPT_H
#define BIPEDALV1_INTERRUPT_H

namespace STM32F411 {
    struct NVICMemoryMap {
        volatile uint32_t ISER[8]; // Offset: 0x000 (Interrupt Set Enable)
        uint32_t RESERVED0[24];
        volatile uint32_t ICER[8]; // Offset: 0x080 (Interrupt Clear Enable)
        uint32_t RESERVED1[24];
        volatile uint32_t ISPR[8]; // Offset: 0x100 (Interrupt Set Pending)
        uint32_t RESERVED2[24];
        volatile uint32_t ICPR[8]; // Offset: 0x180 (Interrupt Clear Pending)
        uint32_t RESERVED3[24];
        volatile uint32_t IABR[8]; // Offset: 0x200 (Interrupt Active bit)
        uint32_t RESERVED4[56];
        volatile uint8_t IP[240]; // Offset: 0x300 (Interrupt Priority - Byte accessible)
    };

    struct EXTIRegMemoryMap {
        volatile uint32_t IMR; // Interrupt mask register
        volatile uint32_t EMR; // Event mask register
        volatile uint32_t RTSR; // Rising trigger selection register
        volatile uint32_t FTSR; // Falling trigger selection register
        volatile uint32_t SWIER; // Software interrupt event register
        volatile uint32_t PR; // Pending register
    };

    inline auto EXTIReg = reinterpret_cast<EXTIRegMemoryMap *>(0x40013C00);

    inline auto NVIC = reinterpret_cast<NVICMemoryMap *>(0xE000E100);
    using InterruptCallback = void (*)();

    struct InterruptManager {
        static inline InterruptCallback callbacks[8] = {nullptr};

        static constexpr uint8_t TCIF_OFFSETS[4] = {5, 11, 21, 27};
        static inline InterruptCallback exti_callbacks[16] = {nullptr};

        enum class EXTITrigger {
            RISING = 0,
            FALLING = 1,
            BOTH=2,
        };
        // A single generic attachDMAInterrupt function
        enum class Stream:uint32_t {
            S0,
            S1,
            S2,
            S3,
            S4,
            S5,
            S6,
        };

        enum class Priority {
            _0,
            _1,
            _2,
            _3,
            _4,
            _5,
            _6,
            _7,
            _8,
            _9,
            _10,
            _11,
            _12,
            _13,
            _14,
            _15,
        };

        enum class EXTILine {
            Line0 = 0, Line1, Line2, Line3, Line4, Line5, Line6, Line7,
            Line8, Line9, Line10, Line11, Line12, Line13, Line14, Line15
        };

        enum class IRQn : uint8_t {
            EXTI0 = 6, // External Interrupt Line 0
            EXTI1 = 7, // External Interrupt Line 1
            EXTI2 = 8, // External Interrupt Line 2
            EXTI3 = 9, // External Interrupt Line 3
            EXTI4 = 10, // External Interrupt Line 4
            DMA1_Stream0 = 11, // DMA1 Stream 0
            DMA1_Stream1 = 12, // DMA1 Stream 1
            DMA1_Stream2 = 13, // DMA1 Stream 2
            DMA1_Stream3 = 14, // DMA1 Stream 3
            DMA1_Stream4 = 15, // DMA1 Stream 4
            DMA1_Stream5 = 16, // DMA1 Stream 5
            DMA1_Stream6 = 17, // DMA1 Stream 6
            EXTI9_5 = 23, // External Interrupt Lines 5 through 9
            EXTI15_10 = 40, // External Interrupt Lines 10 through 15
            DMA1_Stream7 = 47, // DMA1 Stream 7
        };

        enum class EXTISource {
            GPIOA = 0,
            GPIOB,
            GPIOC,
            GPIOD,
            GPIOE,
            GPIOH = 7,
        };

        static void enable(IRQn irq) {
            const auto irq_num = static_cast<uint8_t>(irq);
            NVIC->ISER[irq_num / 32] = 1 << (irq_num % 32);
        }

        static void disable(IRQn irq) {
            const auto irq_num = static_cast<uint8_t>(irq);
            NVIC->ICER[irq_num / 32] = (1 << (irq_num % 32));
        }

        static constexpr IRQn DMA1_STREAM_IRQS[8] = {
            IRQn::DMA1_Stream0, IRQn::DMA1_Stream1, IRQn::DMA1_Stream2, IRQn::DMA1_Stream3,
            IRQn::DMA1_Stream4, IRQn::DMA1_Stream5, IRQn::DMA1_Stream6, IRQn::DMA1_Stream7,
        };

        static void attachDMAInterrupt(Stream stream, InterruptCallback callback) {
            const auto stream_idx = static_cast<uint32_t>(stream);
            callbacks[stream_idx] = callback;
            enable(DMA1_STREAM_IRQS[stream_idx]);
        }

        static void setInterruptPriority(IRQn irq, Priority priority) {
            NVIC->IP[static_cast<uint8_t>(irq)] = (static_cast<uint8_t>(priority) << 4);
        }

        static void attachEXTIInterrupt(EXTILine line, InterruptCallback callback, EXTISource exti_source, EXTITrigger trigger) {
            const auto line_num = static_cast<uint8_t>(line);
            exti_callbacks[line_num] = callback;

            // 1. Find which of the 4 EXTICR registers to use (0, 1, 2, or 3)
            const uint8_t reg_index = line_num / 4;

            // 2. Find the bit shift (0, 4, 8, or 12)
            const uint8_t bit_shift = (line_num % 4) * 4;

            // 3. Clear the 4 specific bits for this line without touching the others
            MemoryMap::SYSCFG->EXTICR[reg_index] &= ~(0xF << bit_shift);

            // 4. Insert the new port source into those 4 bits
            MemoryMap::SYSCFG->EXTICR[reg_index] |= (static_cast<uint32_t>(exti_source) << bit_shift);

            // 5. Unmask this line so the EXTI hardware actually listens to it
            EXTIReg->IMR |= (1 << line_num);

            // Enable the correct NVIC IRQ for this EXTI line
            if (line_num <= 4) {
                enable(static_cast<IRQn>(static_cast<uint8_t>(IRQn::EXTI0) + line_num));
            } else if (line_num <= 9) {
                enable(IRQn::EXTI9_5);
            } else {
                enable(IRQn::EXTI15_10);
            }

            EXTIReg->RTSR |= (1 << line_num);
            if (trigger == EXTITrigger::FALLING) {
                EXTIReg->RTSR &= ~(1 << line_num);
                EXTIReg->FTSR |= (1 << line_num);
            }
            else if (trigger == EXTITrigger::RISING) {
                EXTIReg->FTSR &= ~(1 << line_num);
                EXTIReg->RTSR |= (1 << line_num);
            }
            else if (trigger == EXTITrigger::BOTH) {
                EXTIReg->RTSR |= (1 << line_num);
                EXTIReg->FTSR |= (1 << line_num);
            }
        }
    };
}
#endif //BIPEDALV1_INTERRUPT_H
