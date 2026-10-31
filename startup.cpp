#include "drivers/Clock.h"
#include "drivers/Interrupt.h"
#include "drivers/MemoryMap.h"

using init_func_t = void (*)();

constexpr unsigned int ISRV_LENGTH = 102;

using isr_t = void (*)();

extern "C" void _estack(void);

extern "C" unsigned int
        _sidata,
        _sdata,
        _edata,
        _sbss,
        _ebss;

extern int main(void);

extern "C" init_func_t _sinit;
extern "C" init_func_t _einit;

static void initSystemClock() {
    F411::MemoryMap::RCC1->enableHSE();
    F411::MemoryMap::FlashInterface->setWaitState(F411::MemoryMap::Flash::WaitStates::THREE);
    F411::MemoryMap::RCC1->enablePLL(F411::MemoryMap::RCC::PLLSource::HSE, 25, 192, 2, 4);

    F411::MemoryMap::RCC1->setAPB1PreScaler(F411::MemoryMap::RCC::Prescaler::Half);
    F411::MemoryMap::RCC1->setAPB2PreScaler(F411::MemoryMap::RCC::Prescaler::None);
    F411::MemoryMap::RCC1->setAHBPrescaler(F411::MemoryMap::RCC::AHBPrescaler::None);

    F411::MemoryMap::RCC1->setSystemClockSrc(F411::MemoryMap::RCC::SystemClockSource::PLL);
}

extern "C" [[noreturn]] void Reset_Handler(void) {
    volatile unsigned int *SCB_VTOR = (volatile unsigned int *) 0xE000ED08;
    *SCB_VTOR = 0x08000000;

    volatile unsigned int *SCB_CPACR = (volatile unsigned int *) 0xE000ED88;
    *SCB_CPACR |= 0xF << 20;
    
    asm volatile("dsb" ::: "memory");
    asm volatile("isb" ::: "memory");

    initSystemClock();

    volatile unsigned int *src = &_sidata;
    volatile unsigned int *dst = &_sdata;
    while (dst < &_edata) {
        *dst = *src;
        src++;
        dst++;
    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst = 0;
        dst++;
    }

    init_func_t *src_func = &_sinit;
    while (src_func < &_einit) {
        (*src_func)();
        src_func++;
    }

    main();

    while (1) {
    }
}

static void dma1StreamInterrupt(uint8_t stream) {
    auto dma = F411::MemoryMap::DMA1;
    uint8_t bit_offset = F411::InterruptManager::TCIF_OFFSETS[stream % 4];

    volatile F411::MemoryMap::register_t &status_reg = (stream < 4) ? dma->LISR : dma->HISR;
    volatile F411::MemoryMap::register_t &clear_reg = (stream < 4) ? dma->LIFCR : dma->HIFCR;

    if (status_reg & (1 << bit_offset)) {
        clear_reg = (1 << bit_offset);

        if (F411::InterruptManager::dma_callbacks[stream] != nullptr) {
            F411::InterruptManager::dma_callbacks[stream]();
        }
    }
}

static void handleEXTI(uint8_t start_line, uint8_t end_line) {
    for (uint8_t i = start_line; i <= end_line; i++) {
        if (F411::EXTIReg->PR & (1 << i)) {
            F411::EXTIReg->PR = (1 << i);

            if (F411::InterruptManager::exti_callbacks[i] != nullptr) {
                F411::InterruptManager::exti_callbacks[i]();
            }
        }
    }
}

extern "C" {
void EXTI0_IRQHandler() { handleEXTI(0, 0); }
void EXTI1_IRQHandler() { handleEXTI(1, 1); }
void EXTI2_IRQHandler() { handleEXTI(2, 2); }
void EXTI3_IRQHandler() { handleEXTI(3, 3); }
void EXTI4_IRQHandler() { handleEXTI(4, 4); }

void EXTI9_5_IRQHandler() { handleEXTI(5, 9); }
void EXTI15_10_IRQHandler() { handleEXTI(10, 15); }

void DMA1_Stream0_IRQHandler() { dma1StreamInterrupt(0); }
void DMA1_Stream1_IRQHandler() { dma1StreamInterrupt(1); }
void DMA1_Stream2_IRQHandler() { dma1StreamInterrupt(2); }
void DMA1_Stream3_IRQHandler() { dma1StreamInterrupt(3); }
void DMA1_Stream4_IRQHandler() { dma1StreamInterrupt(4); }
void DMA1_Stream5_IRQHandler() { dma1StreamInterrupt(5); }
void DMA1_Stream6_IRQHandler() { dma1StreamInterrupt(6); }
void DMA1_Stream7_IRQHandler() { dma1StreamInterrupt(7); }
}

extern "C" void Default_Handler(void) {
    while (1) {
    }
}

extern "C" void HardFault_Handler(void) {
    while (1) {
    }
}

extern "C" void TIM1_UP_TIM10_IRQHandler()
{
    using InterruptManager = F411::InterruptManager;

    auto* handlers = InterruptManager::timer_uif_callbacks;

    if (F411::MemoryMap::TIMER10->SR & 0b1) {
        F411::MemoryMap::TIMER10->SR &= ~0b1;

        if (handlers[static_cast<uint8_t>(InterruptManager::Timer::_10)]) {
            handlers[static_cast<uint8_t>(InterruptManager::Timer::_10)]();
        }
    }
}

__attribute__((section(".isr_vector"), used))
isr_t isr_vector_table[ISRV_LENGTH] = {
    _estack,
    Reset_Handler,
    Default_Handler,
    HardFault_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    Default_Handler,
    Default_Handler,
    nullptr,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Stream0_IRQHandler,
    DMA1_Stream1_IRQHandler,
    DMA1_Stream2_IRQHandler,
    DMA1_Stream3_IRQHandler,
    DMA1_Stream4_IRQHandler,
    DMA1_Stream5_IRQHandler,
    DMA1_Stream6_IRQHandler,
    Default_Handler,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    EXTI9_5_IRQHandler,
    Default_Handler,
    TIM1_UP_TIM10_IRQHandler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    nullptr,
    EXTI15_10_IRQHandler,
    Default_Handler,
    Default_Handler,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    Default_Handler,
    nullptr,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    Default_Handler,
    nullptr,
    nullptr,
    Default_Handler,
    Default_Handler
};
