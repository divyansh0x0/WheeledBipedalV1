//
// Created by divyansh on 7/3/26.
//

#ifndef BIPEDALV1_PWM_H
#define BIPEDALV1_PWM_H

#include "Clock.h"
#include "MemoryMap.h"

namespace STM32F411::PWM {
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
        void setFrequency(uint32_t frequency_hz=1000, unsigned int resolution = 10000) {

            const uint32_t system_clock_speed = Clock::getAPB1TimerClock();
            const auto reg = reinterpret_cast<MemoryMap::TIMER *>(timer);
            uint32_t max_freq_hz = system_clock_speed/resolution;
            uint32_t min_freq_hz = system_clock_speed/(65536 * resolution) + 1;
            if (frequency_hz > max_freq_hz) {
                frequency_hz = max_freq_hz;
            }
            if (frequency_hz < min_freq_hz) {
                frequency_hz = min_freq_hz;
            }

            const uint32_t psc_value = (system_clock_speed / (frequency_hz * resolution)) - 1;

            const uint32_t arr_value = resolution - 1;

            // Write to my actual hardware registers
            reg->PSC = psc_value;
            reg->ARR = arr_value;
            reg->EGR |= 0b1 << 0; // reload
        }


        void setDutyCycle(uint32_t duty_cycle) {
            const auto reg = reinterpret_cast<MemoryMap::TIMER *>(timer);
            const uint32_t ccr_value = ((reg->ARR + 1) * duty_cycle) / 100;
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
                reg->CCMR1 &= ~(0b11 << 0); // Configure channel 1 in output compare mode (CC1S = 00 in TIMx_CCMR1)
                reg->CCMR1 &= ~(0b111 << 4); // Clear output compare 1 mode configuration bits (OC1M)
                reg->CCMR1 |= (0b110 << 4); // Set OC1M to PWM Mode 1 (0b110)
                reg->CCMR1 |= (0b1 << 3); // Enable Output Compare 1 Preload (OC1PE)
                reg->CCER |= (0b1 << 0); // Enable Output Compare 1 output (CC1E in TIMx_CCER)
            } else if constexpr (channel == TimerChannel::Channel2) {
                reg->CCMR1 &= ~(0b11 << 8); // Configure channel 2 in output compare mode (CC2S = 00 in TIMx_CCMR1)
                reg->CCMR1 &= ~(0b111 << 12); // Clear output compare 2 mode configuration bits (OC2M)
                reg->CCMR1 |= (0b110 << 12); // Set OC2M to PWM Mode 1 (0b110)
                reg->CCMR1 |= (0b1 << 11); // Enable Output Compare 2 Preload (OC2PE)
                reg->CCER |= (0b1 << 4); // Enable Output Compare 2 output (CC2E in TIMx_CCER)
            } else if constexpr (channel == TimerChannel::Channel3) {
                reg->CCMR2 &= ~(0b11 << 0); // Configure channel 3 in output compare mode (CC3S = 00 in TIMx_CCMR2)
                reg->CCMR2 &= ~(0b111 << 4); // Clear output compare 3 mode configuration bits (OC3M)
                reg->CCMR2 |= (0b110 << 4); // Set OC3M to PWM Mode 1 (0b110)
                reg->CCMR2 |= (0b1 << 3); // Enable Output Compare 3 Preload (OC3PE)
                reg->CCER |= (0b1 << 8); // Enable Output Compare 3 output (CC3E in TIMx_CCER)
            } else if constexpr (channel == TimerChannel::Channel4) {
                reg->CCMR2 &= ~(0b11 << 8); // Configure channel 4 in output compare mode (CC4S = 00 in TIMx_CCMR2)
                reg->CCMR2 &= ~(0b111 << 12); // Clear output compare 4 mode configuration bits (OC4M)
                reg->CCMR2 |= (0b110 << 12); // Set OC4M to PWM Mode 1 (0b110)
                reg->CCMR2 |= (0b1 << 11); // Enable Output Compare 4 Preload (OC4PE)
                reg->CCER |= (0b1 << 12); // Enable Output Compare 4 output (CC4E in TIMx_CCER)
            }
            reg->EGR |= (0b1 << 0); //UG: Update generation
            reg->CR1 |= (0b1 << 7); //ARR enable
            reg->CR1 |= (0b1 << 0); //Enable counter mode
        }

         void disable() {
            const auto reg = reinterpret_cast<MemoryMap::TIMER *>(timer);
            reg->CCER &= ~(0b1 << (static_cast<uint32_t>(channel) * 4));
        }
    };
}
#endif //BIPEDALV1_PWM_H
