#ifndef BIPEDALV1_PWM_H
#define BIPEDALV1_PWM_H

#include <cmath>

#include "Clock.h"
#include "MemoryMap.h"
namespace F411::PWM {
    enum class Timer {
        TIMER2 = 0x4000'0000u,
        TIMER3 = 0x4000'0400u,
        TIMER4 = 0x4000'0800u,
        TIMER5 = 0x4000'0C00u,
    };

    enum class TimerChannel : unsigned int {
        Channel1 = 0,
        Channel2 = 1,
        Channel3 = 2,
        Channel4 = 3
    };

    template<Timer timer, TimerChannel channel>
    class PWM {
    public:
        void setFrequency(uint32_t frequency_hz) {
            const uint32_t system_clock_speed = Clock::getAPB1TimerClock();
            const auto reg = reinterpret_cast<MemoryMap::TIMER *>(timer);
            uint32_t resolution = system_clock_speed / frequency_hz;
            uint32_t min_resolution = system_clock_speed / (65536 * frequency_hz) + 1;
            if (resolution < min_resolution) {
                resolution = min_resolution;
            }
            const uint32_t psc_value = (system_clock_speed / (frequency_hz * resolution)) - 1;
            const uint32_t arr_value = resolution - 1;

            reg->PSC = psc_value;
            reg->ARR = arr_value;
            reg->EGR |= 0b1 << 0;
        }


        void setDutyCycle(float duty_cycle) {
            const auto reg = reinterpret_cast<MemoryMap::TIMER *>(timer);
            const auto ccr_value = static_cast<uint32_t>(
                roundf((static_cast<float>(reg->ARR + 1u) * duty_cycle))
            );
            if constexpr (channel == TimerChannel::Channel1) {
                reg->CCR1 = ccr_value;
            } else if constexpr (channel == TimerChannel::Channel2) {
                reg->CCR2 = ccr_value;
            } else if constexpr (channel == TimerChannel::Channel3) {
                reg->CCR3 = ccr_value;
            } else if constexpr (channel == TimerChannel::Channel4) {
                reg->CCR4 = ccr_value;
            }
        }

        void enable() {
            if constexpr (timer == Timer::TIMER2) {
                MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER2);
            } else if constexpr (timer == Timer::TIMER3) {
                MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER3);
            } else if constexpr (timer == Timer::TIMER4) {
                MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER4);
            } else if constexpr (timer == Timer::TIMER5) {
                MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER5);
            }
            const auto reg = reinterpret_cast<MemoryMap::TIMER *>(timer);

            if constexpr (channel == TimerChannel::Channel1) {
                reg->CCMR1 &= ~(0b11 << 0);
                reg->CCMR1 &= ~(0b111 << 4);
                reg->CCMR1 |= (0b110 << 4);
                reg->CCMR1 |= (0b1 << 3);
                reg->CCER |= (0b1 << 0);
            } else if constexpr (channel == TimerChannel::Channel2) {
                reg->CCMR1 &= ~(0b11 << 8);
                reg->CCMR1 &= ~(0b111 << 12);
                reg->CCMR1 |= (0b110 << 12);
                reg->CCMR1 |= (0b1 << 11);
                reg->CCER |= (0b1 << 4);
            } else if constexpr (channel == TimerChannel::Channel3) {
                reg->CCMR2 &= ~(0b11 << 0);
                reg->CCMR2 &= ~(0b111 << 4);
                reg->CCMR2 |= (0b110 << 4);
                reg->CCMR2 |= (0b1 << 3);
                reg->CCER |= (0b1 << 8);
            } else if constexpr (channel == TimerChannel::Channel4) {
                reg->CCMR2 &= ~(0b11 << 8);
                reg->CCMR2 &= ~(0b111 << 12);
                reg->CCMR2 |= (0b110 << 12);
                reg->CCMR2 |= (0b1 << 11);
                reg->CCER |= (0b1 << 12);
            }
            reg->EGR |= (0b1 << 0);
            reg->CR1 |= (0b1 << 7);
            reg->CR1 |= (0b1 << 0);
        }

        void disable() {
            const auto reg = reinterpret_cast<MemoryMap::TIMER *>(timer);
            reg->CCER &= ~(0b1 << (static_cast<uint32_t>(channel) * 4));
        }
    };
}
#endif //BIPEDALV1_PWM_H
