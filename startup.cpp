#include "drivers/Clock.h"
#include "drivers/Interrupt.h"
#include "drivers/MemoryMap.h"

/**
 * @brief Cortex-M4F vector table for STM32F411.
 *
 * The vector table contains:
 * - The initial stack pointer value.
 * - Exception and interrupt handler addresses.
 */
using init_func_t = void (*)();

/**
 * @brief Total length of the Interrupt Service Routine Vector (ISRV) table for F411.
 * 16 Core Exceptions + 86 Peripheral Interrupts = 102
 */
constexpr unsigned int ISRV_LENGTH = 102;

using isr_t = void (*)();

extern "C" void _estack(void);

extern "C" unsigned int
        _sidata, ///< Start address of the initialization values for the .data section in Flash.
        _sdata, ///< Start address of the .data section in SRAM.
        _edata, ///< End address of the .data section in SRAM.
        _sbss, ///< Start address of the .bss section in SRAM.
        _ebss; ///< End address of the .bss section in SRAM.

extern int main(void);

extern "C" init_func_t _sinit;
extern "C" init_func_t _einit;

/**
 * @brief Configure system clock to 96 MHz using an external 25 MHz HSE crystal.
 * STM32F411 max frequency is 100MHz. 96MHz is chosen to easily generate 48MHz for USB.
 */
static void initSystemClock() {
    // 1. Enable the External Crystal (HSE) and wait for hardware lock.
    STM32F411::MemoryMap::RCC1->enableHSE();

    // 2. Configure Flash Latency FIRST.
    // For STM32F411 at 96MHz (3.3V VDD), 3 Wait States are required.
    STM32F411::MemoryMap::FlashInterface->setWaitState(STM32F411::MemoryMap::Flash::WaitStates::THREE);

    // 3. Configure and enable the PLL.
    // Target: 96 MHz SYSCLK, 48 MHz USB.
    // F4 PLL Formula: f(VCO) = f(HSE) * (N / M). f(SYSCLK) = f(VCO) / P. f(USB) = f(VCO) / Q.
    // M=25 (1MHz VCO in), N=192 (192MHz VCO out), P=2 (96MHz Core), Q=4 (48MHz USB)
    STM32F411::MemoryMap::RCC1->enablePLL(STM32F411::MemoryMap::RCC::PLLSource::HSE, 25, 192, 2, 4);

    // 4. Set bus prescalers BEFORE switching the system clock.
    // AHB  = 96 MHz (Prescaler = None)   -> Max 100 MHz
    // APB1 = 48 MHz (Prescaler = Half)   -> Max 50 MHz
    // APB2 = 96 MHz (Prescaler = None)   -> Max 100 MHz
    STM32F411::MemoryMap::RCC1->setAPB1PreScaler(STM32F411::MemoryMap::RCC::Prescaler::Half);
    STM32F411::MemoryMap::RCC1->setAPB2PreScaler(STM32F411::MemoryMap::RCC::Prescaler::None);
    STM32F411::MemoryMap::RCC1->setAHBPrescaler(STM32F411::MemoryMap::RCC::AHBPrescaler::None);

    // 5. Route the PLL to the Core.
    STM32F411::MemoryMap::RCC1->setSystemClockSrc(STM32F411::MemoryMap::RCC::SystemClockSource::PLL);
    STM32F411::Clock::enable();
}

/**
 * @brief Reset handler called on processor reset.
 */
extern "C" [[noreturn]] void Reset_Handler(void) {
    // Enable Cortex-M4F Hardware Floating Point Unit (FPU)
    // CPACR is located at address 0xE000ED88
    volatile unsigned int *SCB_CPACR = (volatile unsigned int *) 0xE000ED88;
    *SCB_CPACR |= 0xF << 20; // Set CP10 and CP11 to Full Access
    initSystemClock();

    unsigned int *src = &_sidata;
    unsigned int *dst = &_sdata;
    while (dst < &_edata) {
        *dst = *src;
        src++;
        dst++;
    }

    src = &_sbss;
    while (src < &_ebss) {
        *src = 0;
        src++;
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
    auto dma = STM32F411::MemoryMap::DMA1;
    uint8_t bit_offset = STM32F411::InterruptManager::TCIF_OFFSETS[stream % 4];

    // Streams 0-3 use LISR/LIFCR. Streams 4-7 use HISR/HIFCR.
    volatile STM32F411::MemoryMap::register_t &status_reg = (stream < 4) ? dma->LISR : dma->HISR;
    volatile STM32F411::MemoryMap::register_t &clear_reg = (stream < 4) ? dma->LIFCR : dma->HIFCR;

    // Check if the Transfer Complete Interrupt Flag is set
    if (status_reg & (1 << bit_offset)) {
        // Clear the flag to prevent an infinite interrupt loop
        clear_reg = (1 << bit_offset);

        // Execute the dynamically assigned callback
        if (STM32F411::InterruptManager::dma_callbacks[stream] != nullptr) {
            STM32F411::InterruptManager::dma_callbacks[stream]();
        }
    }
}

static void handleEXTI(uint8_t start_line, uint8_t end_line) {
    for (uint8_t i = start_line; i <= end_line; i++) {
        // Check if the Pending Register (PR) flag is set for this specific line
        if (STM32F411::EXTIReg->PR & (1 << i)) {
            // CLEAR the flag by writing a 1 to it (STM32 hardware quirk: rc_w1)
            STM32F411::EXTIReg->PR = (1 << i);

            // Execute the user's callback if it exists
            if (STM32F411::InterruptManager::exti_callbacks[i] != nullptr) {
                STM32F411::InterruptManager::exti_callbacks[i]();
            }
        }
    }
}

extern "C" {
// Dedicated IRQs
void EXTI0_IRQHandler() { handleEXTI(0, 0); }
void EXTI1_IRQHandler() { handleEXTI(1, 1); }
void EXTI2_IRQHandler() { handleEXTI(2, 2); }
void EXTI3_IRQHandler() { handleEXTI(3, 3); }
void EXTI4_IRQHandler() { handleEXTI(4, 4); }

// Grouped IRQs (The loop inside handleEXTI will figure out exactly which pin fired)
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
        // Halt execution for debugger
    }
}

extern "C" void HardFault_Handler(void) {
    while (1) {
        // Halt execution for GDB inspection
    }
}


extern "C" void TIM1_UP_TIM10_IRQHandler()
{
    using InterruptManager = STM32F411::InterruptManager;

    auto* handlers = InterruptManager::timer_uif_callbacks;

    // if (STM32F411::MemoryMap::TIMER1->SR & 0x01) {
    //     STM32F411::MemoryMap::TIMER1->SR &= ~0x01;
    //
    //     if (handlers[static_cast<uint8_t>(InterruptManager::Timer::_1)]) {
    //         handlers[static_cast<uint8_t>(InterruptManager::Timer::_1)]();
    //     }
    // }

    if (STM32F411::MemoryMap::TIMER10->SR & 0b1) {
        STM32F411::MemoryMap::TIMER10->SR &= ~0b1;

        if (handlers[static_cast<uint8_t>(InterruptManager::Timer::_10)]) {
            handlers[static_cast<uint8_t>(InterruptManager::Timer::_10)]();
        }
    }
}

// STM32F411 Vector Table
__attribute__((section(".isr_vector"), used))
isr_t isr_vector_table[ISRV_LENGTH] = {
    _estack, //   0: Initial Stack Pointer
    Reset_Handler, //   1: Reset Vector
    Default_Handler, //   2: NMI (Non-Maskable Interrupt)
    HardFault_Handler, //   3: Hard Fault
    Default_Handler, //   4: Memory Management Fault
    Default_Handler, //   5: Bus Fault
    Default_Handler, //   6: Usage Fault
    nullptr, //   7: Reserved
    nullptr, //   8: Reserved
    nullptr, //   9: Reserved
    nullptr, //  10: Reserved
    Default_Handler, //  11: SVCall
    Default_Handler, //  12: Debug Monitor
    nullptr, //  13: Reserved
    Default_Handler, //  14: PendSV
    Default_Handler, //  15: SysTick

    // External Interrupts
    Default_Handler, //  0: WWDG
    Default_Handler, //  1: PVD
    Default_Handler, //  2: TAMP_STAMP
    Default_Handler, //  3: RTC_WKUP
    Default_Handler, //  4: FLASH
    Default_Handler, //  5: RCC
    EXTI0_IRQHandler, //  6: EXTI0
    EXTI1_IRQHandler, //  7: EXTI1
    EXTI2_IRQHandler, //  8: EXTI2
    EXTI3_IRQHandler, //  9: EXTI3
    EXTI4_IRQHandler, //  10: EXTI4
    DMA1_Stream0_IRQHandler, //  11: DMA1_Stream0
    DMA1_Stream1_IRQHandler, //  12: DMA1_Stream1
    DMA1_Stream2_IRQHandler, //  13: DMA1_Stream2
    DMA1_Stream3_IRQHandler, //  14: DMA1_Stream3
    DMA1_Stream4_IRQHandler, //  15: DMA1_Stream4
    DMA1_Stream5_IRQHandler, //  16: DMA1_Stream5
    DMA1_Stream6_IRQHandler, //  17: DMA1_Stream6
    Default_Handler, // 18: ADC
    nullptr, //  19: Reserved
    nullptr, //  20: Reserved
    nullptr, //  21: Reserved
    nullptr, //  22: Reserved
    EXTI9_5_IRQHandler, //  23: EXTI9_5
    Default_Handler, //  24: TIM1_BRK_TIM9
    TIM1_UP_TIM10_IRQHandler, //  25: TIM1_UP_TIM10
    Default_Handler, //  26: TIM1_TRG_COM_TIM11
    Default_Handler, //  27: TIM1_CC
    Default_Handler, //  28: TIM2
    Default_Handler, //  29: TIM3
    Default_Handler, //  30: TIM4
    Default_Handler, //  31: I2C1_EV
    Default_Handler, //  32: I2C1_ER
    Default_Handler, //  33: I2C2_EV
    Default_Handler, //  34: I2C2_ER
    Default_Handler, //  35: SPI1
    Default_Handler, //  36: SPI2
    Default_Handler, //  37: USART1
    Default_Handler, //  38: USART2
    nullptr, //  39: Reserved
    EXTI15_10_IRQHandler, //  40: EXTI15_10
    Default_Handler, //  41: RTC_Alarm
    Default_Handler, //  42: OTG_FS_WKUP
    nullptr, //  42: Reserved
    nullptr, //  43: Reserved
    nullptr, //  44: Reserved
    nullptr, //  45: Reserved
    Default_Handler, //  45: DMA1_Stream7
    nullptr, //  46: Reserved
    Default_Handler, //  47: SDIO
    Default_Handler, //  48: TIM5
    Default_Handler, //  49: SPI3
    nullptr, //  50: Reserved
    nullptr, //  51: Reserved
    nullptr, //  52: Reserved
    nullptr, //  53: Reserved
    Default_Handler, //  53: DMA2_Stream0
    Default_Handler, //  54: DMA2_Stream1
    Default_Handler, //  55: DMA2_Stream2
    Default_Handler, //  56: DMA2_Stream3
    Default_Handler, //  57: DMA2_Stream4
    nullptr, //  58: Reserved
    nullptr, //  59: Reserved
    nullptr, //  60: Reserved
    nullptr, //  61: Reserved
    nullptr, //  62: Reserved
    nullptr, //  63: Reserved
    Default_Handler, //  63: OTG_FS
    Default_Handler, //  64: DMA2_Stream5
    Default_Handler, //  65: DMA2_Stream6
    Default_Handler, //  66: DMA2_Stream7
    Default_Handler, //  67: USART6
    Default_Handler, //  68: I2C3_EV
    Default_Handler, //  69: I2C3_ER
    nullptr, //  70: Reserved
    nullptr, //  71: Reserved
    nullptr, //  72: Reserved
    nullptr, //  73: Reserved
    nullptr, //  74: Reserved
    nullptr, //  75: Reserved
    nullptr, //  76: Reserved
    Default_Handler, //  77: FPU
    nullptr, //  78: Reserved
    nullptr, //  79: Reserved
    Default_Handler, // 80: SPI4
    Default_Handler // 81: SPI5
};
