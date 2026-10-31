#include "drivers/Clock.h"

namespace F411::Clock {
    static volatile uint64_t COUNTER_RESET_COUNT = 0;
    unsigned int micros() {
        uint32_t cnt = MemoryMap::TIMER10->CNT;
        uint64_t resets = COUNTER_RESET_COUNT;
        
        if ((MemoryMap::TIMER10->SR & 1) && (cnt < 32768)) {
            resets++;
        }
        
        return cnt + (resets * (0xFFFF + 1));
    }

     unsigned int millis() {
        return micros() / 1000;
    }

     unsigned int getSystemClock() {
        unsigned int clock_source = (MemoryMap::RCC1->CFGR & 0x0C) >> 2;

        if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::HSI)) {
            return INTERNAL_CRYSTAL_HZ;
        }
        if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::HSE)) {
            return EXTERNAL_CRYSTAL_HZ;
        }
        if (clock_source == static_cast<unsigned int>(MemoryMap::RCC::SystemClockSource::PLL)) {
            unsigned int pllcfgr = MemoryMap::RCC1->PLLCFGR;

            unsigned int pllsrc_bit = (pllcfgr >> 22) & 0x01;
            unsigned int pll_input_hz = (pllsrc_bit == 1) ? EXTERNAL_CRYSTAL_HZ : 16'000'000;

            unsigned int pllm = pllcfgr & 0x3F;
            unsigned int plln = (pllcfgr >> 6) & 0x1FF;
            unsigned int pllp_bits = (pllcfgr >> 16) & 0x03;
            unsigned int pllp = (pllp_bits + 1) * 2;

            if (pllm == 0 || pllp == 0) return 0;

            unsigned int vco_in = pll_input_hz / pllm;
            unsigned int vco_out = vco_in * plln;
            return vco_out / pllp;
        }
        return 0;
    }

     unsigned int getAHBClock() {
        unsigned int sysclk = getSystemClock();
        unsigned int hpre = (MemoryMap::RCC1->CFGR >> 4) & 0x0F;
        return sysclk / AHB_DIV[hpre];
    }

     unsigned int getAPB1Clock() {
        unsigned int ahb_clk = getAHBClock();
        unsigned int ppre1 = (MemoryMap::RCC1->CFGR >> 10) & 0x07;
        return ahb_clk / APB_DIV[ppre1];
    }

     unsigned int getAPB2Clock() {
        unsigned int ahb_clk = getAHBClock();
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

        MemoryMap::TIMER10->PSC = (getAPB2TimerClock() / 1'000'000) - 1;
        MemoryMap::TIMER10->ARR = 0xFFFF;
        MemoryMap::TIMER10->EGR = (0b1 << 0);
        MemoryMap::TIMER10->SR = ~(0b1 << 0);
        MemoryMap::TIMER10->DIER |= (0b1);
        MemoryMap::TIMER10->CR1 |= (0b1 << 0);

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

        if (APB_DIV[ppre1] > 1) {
            return apb1_clk * 2;
        }
        return apb1_clk;
    }

     unsigned int getAPB2TimerClock() {
        unsigned int ahb_clk = getAHBClock();
        unsigned int ppre2 = (MemoryMap::RCC1->CFGR >> 13) & 0x07;
        unsigned int apb2_clk = ahb_clk / APB_DIV[ppre2];

        if (APB_DIV[ppre2] > 1) {
            return apb2_clk * 2;
        }
        return apb2_clk;
    }
}
