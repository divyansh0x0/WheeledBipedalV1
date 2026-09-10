//
// Created by divyansh on 6/30/26.
//

#ifndef BIPEDALV1_RTC_H
#define BIPEDALV1_RTC_H
#include "Interrupt.h"
#include "MemoryMap.h"
#include<cinttypes>
namespace Biped::Clock {
        static constexpr uint16_t AHB_DIV[16] = {1, 1, 1, 1, 1, 1, 1, 1, 2, 4, 8, 16, 64, 128, 256, 512};
        static constexpr uint8_t APB_DIV[8] = {1, 1, 1, 1, 2, 4, 8, 16};
        static constexpr unsigned int EXTERNAL_CRYSTAL_HZ = 25'000'000;
        static constexpr unsigned int INTERNAL_CRYSTAL_HZ = 16'000'000;

         unsigned int getSystemClock();
         unsigned int getAHBClock();
         unsigned int getAPB1Clock();
         unsigned int getAPB2Clock();
        unsigned int getAPB1TimerClock();
        unsigned int getAPB2TimerClock();
        void enable();

        unsigned int  micros();

        unsigned int  millis();

        void delayMillis(unsigned int duration);
}
#endif //BIPEDALV1_RTC_H
