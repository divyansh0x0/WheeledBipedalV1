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
        _sdata,  ///< Start address of the .data section in SRAM.
        _edata,  ///< End address of the .data section in SRAM.
        _sbss,   ///< Start address of the .bss section in SRAM.
        _ebss;   ///< End address of the .bss section in SRAM.

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
}

/**
 * @brief Reset handler called on processor reset.
 */
extern "C" [[noreturn]] void Reset_Handler(void) {
    // Enable Cortex-M4F Hardware Floating Point Unit (FPU)
    // CPACR is located at address 0xE000ED88
    volatile unsigned int* SCB_CPACR = (volatile unsigned int*)0xE000ED88;
    *SCB_CPACR |= 0xF << 20; // Set CP10 and CP11 to Full Access
    initSystemClock();

    unsigned int* src = &_sidata;
    unsigned int* dst = &_sdata;
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

    init_func_t* src_func = &_sinit;
    while (src_func < &_einit) {
        (*src_func)();
        src_func++;
    }

    main();

    while (1) {}
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

// STM32F411 Vector Table
__attribute__((section(".isr_vector"), used))
isr_t isr_vector_table[ISRV_LENGTH] = {
    _estack,             //   0: Initial Stack Pointer
    Reset_Handler,       //   1: Reset Vector
    Default_Handler,     //   2: NMI (Non-Maskable Interrupt)
    HardFault_Handler,   //   3: Hard Fault
    Default_Handler,     //   4: Memory Management Fault
    Default_Handler,     //   5: Bus Fault
    Default_Handler,     //   6: Usage Fault
    nullptr,             //   7: Reserved
    nullptr,             //   8: Reserved
    nullptr,             //   9: Reserved
    nullptr,             //  10: Reserved
    Default_Handler,     //  11: SVCall
    Default_Handler,     //  12: Debug Monitor
    nullptr,             //  13: Reserved
    Default_Handler,     //  14: PendSV
    Default_Handler,     //  15: SysTick

    // External Interrupts
    Default_Handler,     //  16: WWDG
    Default_Handler,     //  17: PVD
    Default_Handler,     //  18: TAMP_STAMP
    Default_Handler,     //  19: RTC_WKUP
    Default_Handler,     //  20: FLASH
    Default_Handler,     //  21: RCC
    Default_Handler,     //  22: EXTI0
    Default_Handler,     //  23: EXTI1
    Default_Handler,     //  24: EXTI2
    Default_Handler,     //  25: EXTI3
    Default_Handler,     //  26: EXTI4
    Default_Handler,     //  27: DMA1_Stream0
    Default_Handler,     //  28: DMA1_Stream1
    Default_Handler,     //  29: DMA1_Stream2
    Default_Handler,     //  30: DMA1_Stream3
    Default_Handler,     //  31: DMA1_Stream4
    Default_Handler,     //  32: DMA1_Stream5
    Default_Handler,     //  33: DMA1_Stream6
    Default_Handler,     //  34: ADC
    nullptr,             //  35: Reserved
    nullptr,             //  36: Reserved
    nullptr,             //  37: Reserved
    nullptr,             //  38: Reserved
    Default_Handler,     //  39: EXTI9_5
    Default_Handler,     //  40: TIM1_BRK_TIM9
    Default_Handler,     //  41: TIM1_UP_TIM10
    Default_Handler,     //  42: TIM1_TRG_COM_TIM11
    Default_Handler,     //  43: TIM1_CC
    Default_Handler,     //  44: TIM2
    Default_Handler,     //  45: TIM3
    Default_Handler,     //  46: TIM4
    Default_Handler,     //  47: I2C1_EV
    Default_Handler,     //  48: I2C1_ER
    Default_Handler,     //  49: I2C2_EV
    Default_Handler,     //  50: I2C2_ER
    Default_Handler,     //  51: SPI1
    Default_Handler,     //  52: SPI2
    Default_Handler,     //  53: USART1
    Default_Handler,     //  54: USART2
    nullptr,             //  55: Reserved
    Default_Handler,     //  56: EXTI15_10
    Default_Handler,     //  57: RTC_Alarm
    Default_Handler,     //  58: OTG_FS_WKUP
    nullptr,             //  59: Reserved
    nullptr,             //  60: Reserved
    nullptr,             //  61: Reserved
    nullptr,             //  62: Reserved
    Default_Handler,     //  63: DMA1_Stream7
    nullptr,             //  64: Reserved
    Default_Handler,     //  65: SDIO
    Default_Handler,     //  66: TIM5
    Default_Handler,     //  67: SPI3
    nullptr,             //  68: Reserved
    nullptr,             //  69: Reserved
    nullptr,             //  70: Reserved
    nullptr,             //  71: Reserved
    Default_Handler,     //  72: DMA2_Stream0
    Default_Handler,     //  73: DMA2_Stream1
    Default_Handler,     //  74: DMA2_Stream2
    Default_Handler,     //  75: DMA2_Stream3
    Default_Handler,     //  76: DMA2_Stream4
    nullptr,             //  77: Reserved
    nullptr,             //  78: Reserved
    nullptr,             //  79: Reserved
    nullptr,             //  80: Reserved
    nullptr,             //  81: Reserved
    nullptr,             //  82: Reserved
    Default_Handler,     //  83: OTG_FS
    Default_Handler,     //  84: DMA2_Stream5
    Default_Handler,     //  85: DMA2_Stream6
    Default_Handler,     //  86: DMA2_Stream7
    Default_Handler,     //  87: USART6
    Default_Handler,     //  88: I2C3_EV
    Default_Handler,     //  89: I2C3_ER
    nullptr,             //  90: Reserved
    nullptr,             //  91: Reserved
    nullptr,             //  92: Reserved
    nullptr,             //  93: Reserved
    nullptr,             //  94: Reserved
    nullptr,             //  95: Reserved
    nullptr,             //  96: Reserved
    Default_Handler,     //  97: FPU
    nullptr,             //  98: Reserved
    nullptr,             //  99: Reserved
    Default_Handler,     // 100: SPI4
    Default_Handler      // 101: SPI5
};