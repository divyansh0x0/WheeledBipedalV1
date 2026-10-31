#ifndef BIPEDALV1_INTERRUPT_H
#define BIPEDALV1_INTERRUPT_H
#include<cstdint>
#include<drivers/MemoryMap.h>
namespace F411 {
    struct NVICMemoryMap {
        volatile uint32_t ISER[8];
        uint32_t RESERVED0[24];
        volatile uint32_t ICER[8];
        uint32_t RESERVED1[24];
        volatile uint32_t ISPR[8];
        uint32_t RESERVED2[24];
        volatile uint32_t ICPR[8];
        uint32_t RESERVED3[24];
        volatile uint32_t IABR[8];
        uint32_t RESERVED4[56];
        volatile uint8_t IP[240];
    };

    struct EXTIRegMemoryMap {
        volatile uint32_t IMR;
        volatile uint32_t EMR;
        volatile uint32_t RTSR;
        volatile uint32_t FTSR;
        volatile uint32_t SWIER;
        volatile uint32_t PR;
    };

    inline auto EXTIReg = reinterpret_cast<EXTIRegMemoryMap *>(0x40013C00);

    inline auto NVIC = reinterpret_cast<NVICMemoryMap *>(0xE000E100);
    using InterruptCallback = void (*)();

    struct InterruptManager {
        static constexpr uint8_t TCIF_OFFSETS[4] = {5, 11, 21, 27};
        static inline InterruptCallback dma_callbacks[8] = {nullptr};
        static inline InterruptCallback timer_uif_callbacks[10] = {nullptr};
        static inline InterruptCallback exti_callbacks[16] = {nullptr};

        enum class EXTITrigger {
            RISING = 0,
            FALLING = 1,
            BOTH=2,
        };
        enum class DMAStream:uint32_t {
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
            EXTI0 = 6,
            EXTI1 = 7,
            EXTI2 = 8,
            EXTI3 = 9,
            EXTI4 = 10,
            DMA1_Stream0 = 11,
            DMA1_Stream1 = 12,
            DMA1_Stream2 = 13,
            DMA1_Stream3 = 14,
            DMA1_Stream4 = 15,
            DMA1_Stream5 = 16,
            DMA1_Stream6 = 17,
            TIM1_UP_TIM10 = 25,
            EXTI9_5 = 23,
            EXTI15_10 = 40,
            DMA1_Stream7 = 47,
        };
        enum class Timer {
            _1 = 0,
            _2,
            _3,
            _4,
            _5,
            _9 = 8,
            _10 = 9,
            _11 = 10
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

        static void attachTimer10Interrupt(InterruptCallback callback) {
            timer_uif_callbacks[9] = callback;
            setInterruptPriority(IRQn::TIM1_UP_TIM10, Priority::_15);
            enable(IRQn::TIM1_UP_TIM10);
        }


        static constexpr IRQn DMA1_STREAM_IRQS[8] = {
            IRQn::DMA1_Stream0, IRQn::DMA1_Stream1, IRQn::DMA1_Stream2, IRQn::DMA1_Stream3,
            IRQn::DMA1_Stream4, IRQn::DMA1_Stream5, IRQn::DMA1_Stream6, IRQn::DMA1_Stream7,
        };

        static void attachDMAInterrupt(DMAStream stream, InterruptCallback callback) {
            const auto stream_idx = static_cast<uint32_t>(stream);
            dma_callbacks[stream_idx] = callback;
            enable(DMA1_STREAM_IRQS[stream_idx]);
        }

        static void setInterruptPriority(IRQn irq, Priority priority) {
            NVIC->IP[static_cast<uint8_t>(irq)] = (static_cast<uint8_t>(priority) << 4);
        }

        static void attachEXTIInterrupt(EXTILine line, InterruptCallback callback, EXTISource exti_source, EXTITrigger trigger) {
            const auto line_num = static_cast<uint8_t>(line);
            exti_callbacks[line_num] = callback;

            const uint8_t reg_index = line_num / 4;
            const uint8_t bit_shift = (line_num % 4) * 4;

            MemoryMap::SYSCFG->EXTICR[reg_index] &= ~(0xF << bit_shift);
            MemoryMap::SYSCFG->EXTICR[reg_index] |= (static_cast<uint32_t>(exti_source) << bit_shift);

            EXTIReg->PR = (1 << line_num);
            EXTIReg->IMR |= (1 << line_num);

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
