//
// Created by divyansh on 6/30/26.
//

#ifndef BIPEDALV1_RTC_H
#define BIPEDALV1_RTC_H
#include "MemoryMap.h"

namespace STM32F411 {
    class Clock {
        static constexpr uint16_t AHB_DIV[16] = {1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
        static constexpr uint8_t APB_DIV[8] = {1, 1, 1, 1, 2, 4, 8, 16};

    public:
        static constexpr unsigned int EXTERNAL_CRYSTAL_HZ = 25'000'000;
        static constexpr unsigned int INTERNAL_CRYSTAL_HZ = 16'000'000;

        static uint32_t calculateCoreClock() {
            uint32_t clock_source = (MemoryMap::RCC1->CFGR & 0x0C) >> 2; // Read System clock switch status (SWS)

            if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::HSI)) {
                return INTERNAL_CRYSTAL_HZ; // HSI (Internal RC oscillator),  8MHz
            }
            if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::HSE)) {
                return EXTERNAL_CRYSTAL_HZ; // HSE (External oscillator)
            }
            if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::PLL)) {
                uint32_t pllcfgr = MemoryMap::RCC1->PLLCFGR;

                // Bit 22: PLLSRC (0 = HSI, 1 = HSE)
                uint32_t pllsrc_bit = (pllcfgr >> 22) & 0x01;
                uint32_t pll_input_hz = (pllsrc_bit == 1) ? EXTERNAL_CRYSTAL_HZ : 16'000'000;

                // Bits 0-5: PLLM (Division factor for main PLL input clock)
                uint32_t pllm = pllcfgr & 0x3F;

                // Bits 6-14: PLLN (Multiplication factor for VCO)
                uint32_t plln = (pllcfgr >> 6) & 0x1FF;

                // Bits 16-17: PLLP (Division factor for main system clock)
                // Hardware mapping: 00 = /2, 01 = /4, 10 = /6, 11 = /8
                // We can calculate this dynamically: (value + 1) * 2
                uint32_t pllp_bits = (pllcfgr >> 16) & 0x03;
                uint32_t pllp = (pllp_bits + 1) * 2;

                if (pllm == 0 || pllp == 0) return 0; // Prevent division by zero

                // ((Input_Clock / PLLM) * PLLN) / PLLP
                uint32_t vco_in = pll_input_hz / pllm;
                uint32_t vco_out = vco_in * plln;
                return vco_out / pllp;
            }
            return 0; // Error state
        }

        static uint32_t getAHBClock() {
            uint32_t sysclk = calculateCoreClock();
            // HPRE (AHB Prescaler) is at bits 4-7
            uint32_t hpre = (MemoryMap::RCC1->CFGR >> 4) & 0x0F;
            return sysclk / AHB_DIV[hpre];
        }
        static uint32_t getAPB1Clock() {
            uint32_t ahb_clk = getAHBClock();
            // PPRE1 (APB1 Prescaler) is at bits 10-12
            uint32_t ppre1 = (MemoryMap::RCC1->CFGR >> 10) & 0x07;
            return ahb_clk / APB_DIV[ppre1];
        }
        static uint32_t getAPB2Clock() {
            uint32_t ahb_clk = getAHBClock();
            // PPRE2 (APB2 Prescaler) is at bits 13-15
            uint32_t ppre2 = (MemoryMap::RCC1->CFGR >> 13) & 0x07;
            return ahb_clk / APB_DIV[ppre2];
        }
        static uint32_t getAPB1TimerClock() {
            uint32_t ahb_clk = getAHBClock();
            uint32_t ppre1 = (MemoryMap::RCC1->CFGR >> 10) & 0x07;
            uint32_t apb1_clk = ahb_clk / APB_DIV[ppre1];

            // If APB1 prescaler is anything other than 1, hardware doubles the timer clock
            if (APB_DIV[ppre1] > 1) {
                return apb1_clk * 2;
            }
            return apb1_clk;
        }
        static void enable() {
            // 1. Enable TIM5 clock in RCC (Bit 3 in APB1ENR)
            MemoryMap::RCC1->enablePeripheral(MemoryMap::APB1Peripheral::TIMER5);

            // 2. Set the prescaler to get exactly 1 MHz (1 tick = 1 microsecond)
            MemoryMap::TIMER5->PSC = (calculateCoreClock() / 1'000'000) - 1;

            // 3. Set Auto-Reload to max 32-bit value (0xFFFFFFFF)
            MemoryMap::TIMER5->ARR = 0xFFFFFFFF;

            // 4. Generate an update event to load the shadow registers immediately
            MemoryMap::TIMER5->EGR = (1 << 0); // Set UG (Update Generation) bit

            // 5. Enable the timer counter
            MemoryMap::TIMER5->CR1 |= (1 << 0); // Set CEN (Counter Enable) bit
        }

        static uint32_t micros() {
            return MemoryMap::TIMER5->CNT;
        }

        static uint32_t millis() {
            return MemoryMap::TIMER5->CNT / 1000;
        }
    };
}
#endif //BIPEDALV1_RTC_H
