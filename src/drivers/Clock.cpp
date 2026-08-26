#include "drivers/Clock.h"

namespace STM32F411::Clock {
    static volatile uint64_t COUNTER_RESET_COUNT = 0;
    unsigned int micros() {
        return MemoryMap::TIMER10->CNT + (COUNTER_RESET_COUNT * (0xFFFF+1));
    }

     unsigned int millis() {
        return micros() / 1000;
    }

     unsigned int getSystemClock() {
        unsigned int clock_source = (MemoryMap::RCC1->CFGR & 0x0C) >> 2; // Read System clock switch status (SWS)

        if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::HSI)) {
            return INTERNAL_CRYSTAL_HZ; // HSI (Internal RC oscillator),  16MHz
        }
        if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::HSE)) {
            return EXTERNAL_CRYSTAL_HZ; // HSE (External oscillator)
        }
        if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::PLL)) {
            unsigned int pllcfgr = MemoryMap::RCC1->PLLCFGR;

            // Bit 22: PLLSRC (0 = HSI, 1 = HSE)
            unsigned int pllsrc_bit = (pllcfgr >> 22) & 0x01;
            unsigned int pll_input_hz = (pllsrc_bit == 1) ? EXTERNAL_CRYSTAL_HZ : 16'000'000;

            // Bits 0-5: PLLM (Division factor for main PLL input clock)
            unsigned int pllm = pllcfgr & 0x3F;

            // Bits 6-14: PLLN (Multiplication factor for VCO)
            unsigned int plln = (pllcfgr >> 6) & 0x1FF;

            // Bits 16-17: PLLP (Division factor for main system clock)
            // Hardware mapping: 00 = /2, 01 = /4, 10 = /6, 11 = /8
            // We can calculate this dynamically: (value + 1) * 2
            unsigned int pllp_bits = (pllcfgr >> 16) & 0x03;
            unsigned int pllp = (pllp_bits + 1) * 2;

            if (pllm == 0 || pllp == 0) return 0; // Prevent division by zero

            // ((Input_Clock / PLLM) * PLLN) / PLLP
            unsigned int vco_in = pll_input_hz / pllm;
            unsigned int vco_out = vco_in * plln;
            return vco_out / pllp;
        }
        return 0; // Error state
    }

     unsigned int getAHBClock() {
        unsigned int sysclk = getSystemClock();
        // HPRE (AHB Prescaler) is at bits 4-7
        unsigned int hpre = (MemoryMap::RCC1->CFGR >> 4) & 0x0F;
        return sysclk / AHB_DIV[hpre];
    }

     unsigned int getAPB1Clock() {
        unsigned int ahb_clk = getAHBClock();
        // PPRE1 (APB1 Prescaler) is at bits 10-12
        unsigned int ppre1 = (MemoryMap::RCC1->CFGR >> 10) & 0x07;
        return ahb_clk / APB_DIV[ppre1];
    }

     unsigned int getAPB2Clock() {
        unsigned int ahb_clk = getAHBClock();
        // PPRE2 (APB2 Prescaler) is at bits 13-15
        unsigned int ppre2 = (MemoryMap::RCC1->CFGR >> 13) & 0x07;
        return ahb_clk / APB_DIV[ppre2];
    }
    static void updateCounter() {
         COUNTER_RESET_COUNT += 1;
         MemoryMap::TIMER10->DIER |= (0b1 << 0);
     }
     void enable() {
        COUNTER_RESET_COUNT = 0;
        MemoryMap::RCC1->enablePeripheral(MemoryMap::APB2Peripheral::TIMER10);

        // 1. Set the prescaler to get exactly 1 MHz (1 tick = 1ms)
        MemoryMap::TIMER10->PSC = (getAPB2TimerClock() / 1'000'000) - 1;
        // 2. Set Auto-Reload to max 16-bit value (0xFFFFFFFF)
        MemoryMap::TIMER10->ARR = 0xFFFF;

        // 3. Generate an update event to load the shadow registers immediately
        MemoryMap::TIMER10->EGR = (0b1 << 0); // Set UG (Update Generation) bit
         // 4. CLEAR the UIF (Update Interrupt Flag) that was just triggered by EGR
         MemoryMap::TIMER10->SR = ~(0b1 << 0);

         // 5. NOW enable the update interrupt (UIE)
         MemoryMap::TIMER10->DIER |= (0b1);

         // 6. Enable the timer counter
        MemoryMap::TIMER10->CR1 |= (0b1 << 0); // Set CEN (Counter Enable) bit

        InterruptManager::attachTimer10Interrupt(&updateCounter);
    }

     void delayMillis(unsigned int duration) {
        const unsigned int t1 = millis();
        while (millis() - t1 < duration) {
        }
    }

     unsigned int getAPB1TimerClock() {
        unsigned int ahb_clk = getAHBClock();
        unsigned int ppre1 = (MemoryMap::RCC1->CFGR >> 10) & 0x07;
        unsigned int apb1_clk = ahb_clk / APB_DIV[ppre1];

        // If APB1 prescaler is anything other than 1, hardware doubles the timer clock
        if (APB_DIV[ppre1] > 1) {
            return apb1_clk * 2;
        }
        return apb1_clk;
    }

     unsigned int getAPB2TimerClock() {
        unsigned int ahb_clk = getAHBClock();
        unsigned int ppre2 = (MemoryMap::RCC1->CFGR >> 13) & 0x07;
        unsigned int apb2_clk = ahb_clk / APB_DIV[ppre2];

        // If APB2 prescaler is anything other than 1, hardware doubles the timer clock
        if (APB_DIV[ppre2] > 1) {
            return apb2_clk * 2;
        }
        return apb2_clk;
    }
}
