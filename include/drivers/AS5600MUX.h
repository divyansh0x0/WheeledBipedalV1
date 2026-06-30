//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_AS5600_H
#define BIPEDALV1_AS5600_H
#include "MemoryMap.h"
#include "I2C.h"

namespace STM32 {
    template<typename I2C>
    class AS5600MUX {
        AS5600MUX(unsigned int count) {
            if(count < 8)
                count = 8;

            dma.STREAMS[stream_index]
        }
    };
}
#endif //BIPEDALV1_AS5600_H
