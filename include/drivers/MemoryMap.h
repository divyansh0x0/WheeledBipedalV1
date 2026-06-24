/**
 * @file MemoryMap.h
 * @brief Bare-metal peripheral register memory mapping and hardware abstractions for STM32F103.
 * @details This file maps peripheral registers to C++ structure representations and provides
 * inline register manipulation functions for safe register access.
 */

#ifndef WHEEL2FIRMWARE_MEMORYMAP_H
#define WHEEL2FIRMWARE_MEMORYMAP_H

#include <cstdint>

namespace STM32::MemoryMap {
    inline constexpr unsigned int CPU_FREQUENCY = 72'000'000;
    using register_t = unsigned int;

    /**
     * @brief Determines the internal pull-up/pull-down resistor connection for an input pin.
     */


    enum class BaudRate : unsigned int {
        Baud9600 = 9600,
        Baud19200 = 19200,
        Baud38400 = 38400,
        Baud57600 = 57600,
        Baud115200 = 115200,
        Baud230400 = 230400
    };

    enum class AHB1Peripheral: unsigned int {
        GPIOA = 0,
        GPIOB = 1,
        GPIOC = 2,
        GPIOD = 3,
        GPIOE = 4,
        GPIOH = 5,
        DMA1 = 21,
        DMA2 = 22
    };

    enum class APB1Peripheral:unsigned int {
        TIMER2 = 0,
        TIMER3 = 1,
        TIMER4 = 2,
        TIMER5 = 3,
        SPI2 = 14,
        SPI3 = 15,
        USART2 = 17,
        I2C1 = 21,
        I2C2 = 22,
        I2C3 = 23
    };

    /**
     * @brief Identifies peripherals connected to the High-Speed APB2 Bus.
     * @details Mapped to specific bit positions in the RCC_APB2ENR register.
     */
    enum class APB2Peripheral : unsigned int {
        TIMER1 = 0,
        USART1 = 4,
        USART6 = 5,
        ADC1 = 8,
        SDIO = 11,
        SPI1 = 12,

        SPI4 = 13
    };


    /**
     * @brief Identifies the 4 independent hardware channels inside a General Purpose Timer.
     */
    enum class TimerChannel : unsigned int {
        Channel1 = 1,
        Channel2 = 2,
        Channel3 = 3,
        Channel4 = 4
    };

    inline void waitForBit(volatile register_t *reg, const unsigned int bit, const bool state) {
        if (state) {
            while (!(*reg & (1u << bit))) {
            }
        } else {
            while (*reg & (1u << bit)) {
            }
        }
    }


    struct Flash {
        volatile register_t ACR;
        volatile register_t KEYR;
        volatile register_t OPTKEYR;
        volatile register_t SR;
        volatile register_t CR;
        volatile register_t OPTCR;

        enum class WaitStates:unsigned int {
            ZERO = 0b00,
            ONE = 0b01,
            TWO = 0b10,
            THREE = 0b11
        };

        void setWaitState(WaitStates state) {
            unsigned int latency_pin = 0;
            this->ACR = ((this->ACR & ~0b1111) | static_cast<unsigned int>(state)) << latency_pin;
        }
    };

    /**
     * @brief General-purpose timer peripheral register map.
     * @details Represents the register layout for TIM2, TIM3, and TIM4.
     * Offsets map exactly to Section 15.4 of the RM0008 Reference Manual.
     * Read and write accesses must conform to the alignment and data width specifications
     * described in Section 3.1.
     */
    struct TIMER {
        volatile register_t CR1; ///< 0x00 TIMx Control Register 1 (TIMx_CR1).
        volatile register_t CR2; ///< 0x04 TIMx Control Register 2 (TIMx_CR2).
        volatile register_t SMCR; ///< 0x08 TIMx Slave Mode Control Register (TIMx_SMCR).
        volatile register_t DIER; ///< 0x0C TIMx DMA/Interrupt Enable Register (TIMx_DIER).
        volatile register_t SR; ///< 0x10 TIMx Status Register (TIMx_SR).
        volatile register_t EGR; ///< 0x14 TIMx Event Generation Register (TIMx_EGR).
        volatile register_t CCMR1; ///< 0x18 TIMx Capture/Compare Mode Register 1 (TIMx_CCMR1).
        volatile register_t CCMR2; ///< 0x1C TIMx Capture/Compare Mode Register 2 (TIMx_CCMR2).
        volatile register_t CCER; ///< 0x20 TIMx Capture/Compare Enable Register (TIMx_CCER).
        volatile register_t CNT; ///< 0x24 TIMx Counter Register (TIMx_CNT).
        volatile register_t PSC; ///< 0x28 TIMx Prescaler Register (TIMx_PSC).
        volatile register_t ARR; ///< 0x2C TIMx Auto-Reload Register (TIMx_ARR).
        volatile register_t RCR; ///< 0x30 TIMx Repetition Counter Register (TIMx_RCR).
        volatile register_t CCR1; ///< 0x34 TIMx Capture/Compare Register 1 (TIMx_CCR1).
        volatile register_t CCR2; ///< 0x38 TIMx Capture/Compare Register 2 (TIMx_CCR2).
        volatile register_t CCR3; ///< 0x3C TIMx Capture/Compare Register 3 (TIMx_CCR3).
        volatile register_t CCR4; ///< 0x40 TIMx Capture/Compare Register 4 (TIMx_CCR4).
        /**
         * @brief Configures the prescaler and auto-reload values of the timer.
         * @param prescaler Clock division factor (value written to PSC is prescaler - 1).
         * @param auto_reload Period count limit value (value written to ARR is auto_reload - 1).
         */
        // Target frequency in Hz, and your desired max value for 100% duty cycle
        void setFrequency(uint32_t target_hz, uint32_t resolution = 1000) {
            uint32_t system_clock = 72000000; // 72 MHz

            // Formula: PSC = SystemClock / (Frequency * ARR)
            // We subtract 1 because hardware registers are 0-indexed
            uint32_t psc_value = (system_clock / (target_hz * resolution)) - 1;
            uint32_t arr_value = resolution - 1;

            // Write to my actual hardware registers
            this->PSC = psc_value;
            this->ARR = arr_value;
        }

        /**
         * @brief Configures a specific channel for PWM Mode 1 and enables its output.
         * @param channel The target timer channel (1 to 4).
         */
        void enablePWM(TimerChannel channel) {
            switch (channel) {
                case TimerChannel::Channel1:
                    this->CCMR1 &= ~(0b11 << 0); // Configure channel 1 in output compare mode (CC1S = 00 in TIMx_CCMR1)
                    this->CCMR1 &= ~(0b111 << 4); // Clear output compare 1 mode configuration bits (OC1M)
                    this->CCMR1 |= (0b110 << 4); // Set OC1M to PWM Mode 1 (0b110)
                    this->CCMR1 |= (1 << 3); // Enable Output Compare 1 Preload (OC1PE)
                    this->CCER |= (1 << 0); // Enable Output Compare 1 output (CC1E in TIMx_CCER)
                    break;
                case TimerChannel::Channel2:
                    this->CCMR1 &= ~(0b11 << 8); // Configure channel 2 in output compare mode (CC2S = 00 in TIMx_CCMR1)
                    this->CCMR1 &= ~(0b111 << 12); // Clear output compare 2 mode configuration bits (OC2M)
                    this->CCMR1 |= (0b110 << 12); // Set OC2M to PWM Mode 1 (0b110)
                    this->CCMR1 |= (1 << 11); // Enable Output Compare 2 Preload (OC2PE)
                    this->CCER |= (1 << 4); // Enable Output Compare 2 output (CC2E in TIMx_CCER)
                    break;
                case TimerChannel::Channel3:
                    this->CCMR2 &= ~(0b11 << 0); // Configure channel 3 in output compare mode (CC3S = 00 in TIMx_CCMR2)
                    this->CCMR2 &= ~(0b111 << 4); // Clear output compare 3 mode configuration bits (OC3M)
                    this->CCMR2 |= (0b110 << 4); // Set OC3M to PWM Mode 1 (0b110)
                    this->CCMR2 |= (1 << 3); // Enable Output Compare 3 Preload (OC3PE)
                    this->CCER |= (1 << 8); // Enable Output Compare 3 output (CC3E in TIMx_CCER)
                    break;
                case TimerChannel::Channel4:
                    this->CCMR2 &= ~(0b11 << 8); // Configure channel 4 in output compare mode (CC4S = 00 in TIMx_CCMR2)
                    this->CCMR2 &= ~(0b111 << 12); // Clear output compare 4 mode configuration bits (OC4M)
                    this->CCMR2 |= (0b110 << 12); // Set OC4M to PWM Mode 1 (0b110)
                    this->CCMR2 |= (1 << 11); // Enable Output Compare 4 Preload (OC4PE)
                    this->CCER |= (1 << 12); // Enable Output Compare 4 output (CC4E in TIMx_CCER)
                    break;
            }
        }

        /**
         * @brief Updates the duty cycle value for a specific timer channel.
         * @param channel The target timer channel (1 to 4).
         * @param value The compare value written to the Capture/Compare Register (CCR).
         */
        void setDutyCycle(TimerChannel channel, unsigned int value) {
            switch (channel) {
                case TimerChannel::Channel1: this->CCR1 = value;
                    break;
                case TimerChannel::Channel2: this->CCR2 = value;
                    break;
                case TimerChannel::Channel3: this->CCR3 = value;
                    break;
                case TimerChannel::Channel4: this->CCR4 = value;
                    break;
            }
        }

        /**
         * @brief Enables the timer counter.
         */
        void start() {
            this->CR1 |= (1 << 0); // Set CEN (Counter Enable) bit in TIMx_CR1
        }
    };

    static_assert(sizeof(TIMER) == 0x44);

    /**
     * @brief Reset and Clock Control (RCC) peripheral memory map.
     * @details Mapped to physical memory offsets specified in Section 7.3 of the RM0008 Reference Manual.
     * Manages peripheral reset state and clock enable/disable settings.
     */
    struct RCC {
        volatile register_t CR; ///< 0x00 Clock Control Register (RCC_CR).
        volatile register_t PLLCFGR; ///< 0x04 Clock Configuration Register (RCC_CFGR).
        volatile register_t CFGR; ///< 0x04 Clock Configuration Register (RCC_CFGR).
        volatile register_t CIR; ///< 0x08 Clock Interrupt Register (RCC_CIR).
        volatile register_t AHB1RSTR; ///< 0x0C APB2 Peripheral Reset Register (RCC_APB2RSTR).
        volatile register_t AHB2RSTR; ///< 0x0C APB2 Peripheral Reset Register (RCC_APB2RSTR).
        volatile register_t __reserved1;
        volatile register_t __reserved2;
        volatile register_t APB1RSTR; ///< 0x0C APB1 Peripheral Reset Register (RCC_APB2RSTR).
        volatile register_t APB2RSTR; ///< 0x10 APB2 Peripheral Reset Register (RCC_APB1RSTR).
        volatile register_t __reserved3;
        volatile register_t __reserved4;
        volatile register_t AHB1ENR; ///< 0x14 AHB Peripheral Clock Enable Register (RCC_AHBENR).
        volatile register_t AHB2ENR; ///< 0x14 AHB Peripheral Clock Enable Register (RCC_AHBENR).
        volatile register_t __reserved5;
        volatile register_t __reserved6;
        volatile register_t APB1ENR; ///< 0x18 APB2 Peripheral Clock Enable Register (RCC_APB2ENR).
        volatile register_t APB2ENR; ///< 0x1C APB1 Peripheral Clock Enable Register (RCC_APB1ENR).
        volatile register_t __reserved7;
        volatile register_t __reserved8;
        volatile register_t AHB1LPENR; ///< 0x20 Backup Domain Control Register (RCC_BDCR).
        volatile register_t AHB2LPENR; ///< 0x20 Backup Domain Control Register (RCC_BDCR).
        volatile register_t __reserved9;
        volatile register_t __reserved10;
        volatile register_t APB1LPENR; ///< 0x20 Backup Domain Control Register (RCC_BDCR).
        volatile register_t APB2LPENR; ///< 0x20 Backup Domain Control Register (RCC_BDCR).
        volatile register_t __reserved11;
        volatile register_t __reserved12;
        volatile register_t BDCR; ///< 0x20 Backup Domain Control Register (RCC_BDCR).
        volatile register_t CSR; ///< 0x24 Control/Status Register (RCC_CSR).
        volatile register_t __reserved13;
        volatile register_t __reserved14;
        volatile register_t SSCGR; ///< 0x24 Control/Status Register (RCC_CSR).
        volatile register_t PLLI2SCFGR; ///< 0x24 Control/Status Register (RCC_CSR).
        enum class AHBPrescaler {
            None = 0b000,
            Half = 0b1000,
            Quarter = 0b1001,
            Eighth = 0b1010,
            Sixteenth = 0b1011,
        };

        enum class Prescaler {
            None = 0b000,
            Half = 0b100,
            Quarter = 0b101,
            Eighth = 0b110,
            Sixteenth = 0b111,
        };

        enum class PLLSource {
            HSI = 0,
            HSE = 1
        };

        enum class SystemClockSource {
            HSI = 0b00,
            HSE = 0b01,
            PLL = 0b10
        };

        void enableHSI() {
            constexpr unsigned int hsi_on_bit = 0;
            constexpr unsigned int hsi_ready_bit = 1;
            this->CR |= (1 << hsi_on_bit);
            waitForBit(&this->CR, hsi_ready_bit, true);
        }

        void enableHSE() {
            constexpr unsigned int hse_on_bit = 16;
            constexpr unsigned int hse_ready_bit = 17;
            this->CR |= (1 << hse_on_bit);
            waitForBit(&this->CR, hse_ready_bit, true);
        }

        /**
         * @brief Configures and enables the Main PLL for the STM32F411.
         * * @param source The input clock source (HSI or HSE).
         * @param m Division factor for the input clock (2 to 63).
         * @param n Multiplication factor for the VCO (50 to 432).
         * @param p Division factor for the main system clock (2, 4, 6, or 8).
         * @param q Division factor for USB/SDIO (2 to 15).
         */
        void enablePLL(PLLSource source, uint32_t m, uint32_t n, uint32_t p, uint32_t q) {
            constexpr unsigned int pll_on_bit = 24;
            constexpr unsigned int pll_ready_bit = 25;

            // 1. Disable the PLL. Hardware forbids modifying PLLCFGR while PLL is active.
            this->CR &= ~(1 << pll_on_bit);

            // Wait for the analog circuitry to fully power down and release the lock.
            // (Assuming your waitForBit function takes 'false' to wait for a bit to clear).
            waitForBit(&this->CR, pll_ready_bit, false);

            // 2. Read current PLLCFGR state
            uint32_t pllcfgr = this->PLLCFGR;

            // 3. Clear the target bit fields
            // M: bits 5:0 (0x3F)
            // N: bits 14:6 (0x1FF << 6)
            // P: bits 17:16 (0x3 << 16)
            // SRC: bit 22 (1 << 22)
            // Q: bits 27:24 (0xF << 24)
            pllcfgr &= ~((0x3F << 0) | (0x1FF << 6) | (0x3 << 16) | (1 << 22) | (0xF << 24));

            // 4. Calculate P bits. Hardware maps 2->0b00, 4->0b01, 6->0b10, 8->0b11.
            uint32_t p_bits = (p / 2) - 1;

            // 5. Inject new values
            pllcfgr |= (m << 0);
            pllcfgr |= (n << 6);
            pllcfgr |= (p_bits << 16);
            pllcfgr |= (static_cast<uint32_t>(source) << 22);
            pllcfgr |= (q << 24);

            // 6. Write back to hardware
            this->PLLCFGR = pllcfgr;

            // 7. Enable the PLL
            this->CR |= (1 << pll_on_bit);

            // 8. Wait for the hardware to assert the lock
            waitForBit(&this->CR, pll_ready_bit, true);
        }

        void setSystemClockSrc(SystemClockSource source) {
            const unsigned int sw_val = static_cast<unsigned int>(source);

            // Set SW bits (bits 1:0 of RCC_CFGR)
            this->CFGR = (this->CFGR & ~0b11) | sw_val;


            // Block execution until the multiplexer has physically routed the signal
            while ((this->CFGR & (0b11 << 2)) != (sw_val << 2));
        }

        void setAPB1PreScaler(Prescaler scale) {
            constexpr unsigned int ppre_bit = 10;
            this->CFGR &= ~(0b111 << ppre_bit); //reset
            this->CFGR |= static_cast<unsigned int>(scale) << ppre_bit; //set
        }

        void setAPB2PreScaler(Prescaler scale) {
            constexpr unsigned int ppre_bit = 13;
            this->CFGR &= ~(0b111 << ppre_bit); //reset
            this->CFGR |= static_cast<unsigned int>(scale) << ppre_bit; //set
        }

        /**
         * @brief Enables the clock for a peripheral on the APB2 bus.
         * @param peripheral The target APB2 peripheral bit offset value.
         */
        void enablePeripheral(APB2Peripheral peripheral) {
            this->APB2ENR |= (1 << static_cast<unsigned int>(peripheral));
        }

        /**
         * @brief Enables the clock for a peripheral on the APB1 bus.
         * @param peripheral The target APB1 peripheral bit offset value.
         */
        void enablePeripheral(APB1Peripheral peripheral) {
            this->APB1ENR |= (1 << static_cast<unsigned int>(peripheral));
        }

        /**
         * @brief Enables the clock for a peripheral on the AHB1 bus.
         * @param peripheral The target AHB1 peripheral bit offset value.
         */
        void enablePeripheral(AHB1Peripheral peripheral) {
            this->AHB1ENR |= (1 << static_cast<unsigned int>(peripheral));
        }
        
        void setAHBPrescaler(AHBPrescaler scale) {
            const unsigned int hpre_bit = 4;
            this->CFGR &= ~(0b1111 << hpre_bit); //reset
            this->CFGR |= static_cast<unsigned int>(scale) << hpre_bit;
        }
    };


    /**
     * @brief General Purpose Input/Output (GPIO) peripheral memory map.
     * @details Mapped to physical memory offsets specified in Section 9.2 of the RM0008 Reference Manual.
     */
    struct GPIO {
        volatile register_t MODER;   ///< 0x00 Port mode register.
        volatile register_t OTYPER;  ///< 0x04 Port output type register.
        volatile register_t OSPEEDR; ///< 0x08 Port output speed register.
        volatile register_t PUPDR;   ///< 0x0C Port pull-up/pull-down register.
        volatile register_t IDR;     ///< 0x10 Port input data register.
        volatile register_t ODR;     ///< 0x14 Port output data register.
        volatile register_t BSRR;    ///< 0x18 Port bit set/reset register.
        volatile register_t LCKR;    ///< 0x1C Port configuration lock register.
        volatile register_t AFRL;    ///< 0x20 Alternate function low register for pin 0 to 7 on same port.
        volatile register_t AFRH;    ///< 0x24 Alternate function high register for pin 8 to 15 on same port.
        /**
         * @brief Defines the 4-bit configuration for the GPIO MODE and CNF register bitfields.
         * @details Mapped to the CNF[1:0] and MODE[1:0] bitfields in the GPIOx_CRL and GPIOx_CRH registers.
         */
        enum class Mode : uint32_t {
            Input = 0b00,
            Output = 0b01,
            AlternateFunction = 0b10,
            Analog = 0b11
        };

        enum class OutputType : uint32_t {
            PushPull = 0b0,
            OpenDrain = 0b1
        };

        enum class OutputSpeed : uint32_t {
            Low_2MHz = 0b00,
            Medium_25MHz = 0b01,
            Fast_50MHz = 0b10,
            High_100MHz = 0b11   // Very High Speed
        };

        enum class Pull : uint32_t {
            None = 0b00,
            Up = 0b01,
            Down = 0b10
        };
        /**
        * @brief Drives the physical pin to VDD (Logic 1).
                 * @details Uses the lower 16 bits (BSy) of the BSRR for atomic, single-cycle access.
                 */
        void setPinHigh(unsigned int pin_number) {
            this->BSRR = (1u << pin_number);
        }

        /**
         * @brief Drives the physical pin to VSS (Logic 0).
         * @details Uses the upper 16 bits (BRy) of the BSRR for atomic, single-cycle access.
         * The F411 does not have a separate BRR register.
         */
        void setPinLow(unsigned int pin_number) {
            this->BSRR = (1u << (pin_number + 16));
        }

        /**
                 * @brief Configures the entire physical state of a GPIO pin.
                 * @param pin The target pin number (0 to 15).
                 * @param mode Input, Output, Alternate Function, or Analog.
                 * @param pull Physical resistor state (defaults to None).
                 * @param speed Slew rate of the output transistors (defaults to Low).
                 * @param type Push-pull or open-drain (defaults to PushPull).
                 */
        void configurePin(unsigned int pin,
                          Mode mode,
                          Pull pull = Pull::None,
                          OutputSpeed speed = OutputSpeed::Low_2MHz,
                          OutputType type = OutputType::PushPull)
        {
            // 2-bit fields require shifting by (pin * 2)
            const unsigned int shift2 = pin * 2;
            const uint32_t mask2 = 0b11;

            // 1-bit fields require shifting by (pin)
            const unsigned int shift1 = pin;
            const uint32_t mask1 = 0b1;

            // 1. Configure Mode (MODER)
            uint32_t moder = this->MODER;
            moder &= ~(mask2 << shift2);
            moder |= (static_cast<uint32_t>(mode) << shift2);
            this->MODER = moder;

            // 2. Configure Pull Resistors (PUPDR)
            uint32_t pupdr = this->PUPDR;
            pupdr &= ~(mask2 << shift2);
            pupdr |= (static_cast<uint32_t>(pull) << shift2);
            this->PUPDR = pupdr;

            // 3. Configure Output Speed (OSPEEDR)
            uint32_t ospeedr = this->OSPEEDR;
            ospeedr &= ~(mask2 << shift2);
            ospeedr |= (static_cast<uint32_t>(speed) << shift2);
            this->OSPEEDR = ospeedr;

            // 4. Configure Output Type (OTYPER)
            uint32_t otyper = this->OTYPER;
            otyper &= ~(mask1 << shift1);
            otyper |= (static_cast<uint32_t>(type) << shift1);
            this->OTYPER = otyper;
        }
    };


    /***
     * @brief Universal Synchronous Asynchronous Receiver Transmitter (USART) peripheral memory map.
     * @details Mapped to physical memory offsets specified in Section 27.6 of the RM0008 Reference Manual.
     */
    struct USART {
        volatile register_t SR; ///< 0x00 USART Status Register (USART_SR).
        volatile register_t DR; ///< 0x04 USART Data Register (USART_DR).
        volatile register_t BRR; ///< 0x08 USART Baud Rate Register (USART_BRR).
        volatile register_t CR1; ///< 0x0C USART Control Register 1 (USART_CR1).
        volatile register_t CR2; ///< 0x10 USART Control Register 2 (USART_CR2).
        volatile register_t CR3; ///< 0x14 USART Control Register 3 (USART_CR3).
        volatile register_t GTPR; ///< 0x18 USART Guard Time and Prescaler Register (USART_GTPR).

        /**
         * @brief Initializes USART parameters, baud rate division, and enables receiver and transmitter.
         * @param baud Selected transmission BaudRate enum.
         * @param peripheral_clock Frequency of the clock source feeding the USART peripheral.
         */
        void init(BaudRate baud, unsigned int peripheral_clock) {
            // 1. Convert BaudRate enum to integer value
            const auto baud_val = static_cast<unsigned int>(baud);

            // 2. Compute the baud rate division values per Section 27.3.4 of the RM0008 Reference Manual
            const unsigned int usartdiv = (peripheral_clock * 10) / (16 * baud_val);
            const unsigned int mantissa = usartdiv / 10;
            const unsigned int fraction = ((usartdiv % 10) * 16 + 5) / 10;

            this->BRR = (mantissa << 4) | (fraction & 0x0F);

            // 3. Set UE, TE, and RE bits in USART_CR1 register to enable peripheral operations
            this->CR1 = (1 << 13) | (1 << 3) | (1 << 2);
        }

        /**
         * @brief Reads a single received byte. Block until data register not empty.
         * @return Received character byte.
         */
        unsigned char readByte() const {
            while (!(this->SR & (1 << 5))); // Wait until RXNE (Read Data Register Not Empty) bit is set
            return static_cast<unsigned char>(this->DR & 0xFF);
        }

        /**
         * @brief Transmits a single byte. Blocks until transmission register empty.
         * @param data Byte value to transmit.
         */
        void transmit(uint8_t data) {
            while (!(this->SR & (1 << 7))); // Wait until TXE (Transmit Data Register Empty) bit is set
            this->DR = data;
        }

        /**
         * @brief Checks if received data is available to read.
         * @return True if RXNE flag is set, false otherwise.
         */
        bool hasData() const {
            return (this->SR & (1 << 5)); // Check RXNE flag status in USART_SR
        }

        /**
         * @brief Reads the byte currently in the data register without blocking.
         * @return Data register byte value.
         */
        uint8_t receive() const {
            return static_cast<uint8_t>(this->DR & 0xFF);
        }
    };

    struct I2C {
        volatile register_t CR1;
        volatile register_t CR2;
        volatile register_t OAR1;
        volatile register_t OAR2;
        volatile register_t DR;
        volatile register_t SR1;
        volatile register_t SR2;
        volatile register_t CCR;
        volatile register_t TRISE;
    };

    struct DMAChannel {
        volatile register_t CCR; //!< DMA channel x configuration register
        volatile register_t CNDTR; //!< DMA channel x number of data register
        volatile register_t CPAR; //!< DMA channel x peripheral address register
        volatile register_t CMAR; //!< DMA channel x memory address register
        register_t RESERVED; //!< Reserved for alignment. DO NOT WRITE TO IT (13.4.7 Register Map)

        enum class DMAPriorityLevel : unsigned char {
            LOW = 0b00,
            MEDIUM = 0b01,
            HIGH = 0b10,
            VERY_HIGH = 0b11
        };

        enum class DMAMemorySize : unsigned char {
            BYTE = 0b00,
            HALF_WORD = 0b01,
            WORD = 0b10,
        };

        enum class TransferDirection : unsigned char {
            FROM_PERIPHERAL_TO_MEMORY = 0b0,
            FROM_MEMORY_TO_PERIPHERAL = 0b1,
        };

        bool isEnabled() const {
            return CCR & 0b1;
        }

        void setPriorityLevel(DMAPriorityLevel level) {
            constexpr int bit_start = 12;
            // Clear last level
            CCR &= ~(0b11 << bit_start);
            // Set level
            CCR |= (static_cast<register_t>(level) << bit_start);
        }

        void setSize(DMAMemorySize memory_size, DMAMemorySize peripheral_size) {
            // set memory size
            constexpr int m_bit_start = 10;
            CCR &= ~(0b11 << m_bit_start);
            // Set level
            CCR |= (static_cast<register_t>(memory_size) << m_bit_start);

            // set peripheral size
            constexpr int p_bit_start = 8;
            CCR &= ~(0b11 << p_bit_start);
            // Set level
            CCR |= (static_cast<register_t>(peripheral_size) << p_bit_start);
        }

        void setPeripheralAddress(const uintptr_t *peripheral) {
            if (isEnabled())
                return;
            CPAR = reinterpret_cast<register_t>(peripheral);
        }

        void setMemoryAddress(const uintptr_t *memory) {
            if (isEnabled())
                return;
            CMAR = reinterpret_cast<register_t>(memory);
        }

        void setDataTransferMode(const TransferDirection dir) {
            constexpr unsigned int bit_start = 4;
            CCR &= ~(0b1 << bit_start);
            CCR |= (static_cast<register_t>(dir) << bit_start);
        }

        void enableTransferCompleteInterrupt(bool enabled) {
            constexpr unsigned int bit_start = 1;
            CCR &= ~(0b1 << bit_start);
            CCR |= (static_cast<register_t>(enabled) << bit_start);
        }
    };

    struct DMA1MemoryMap {
        volatile register_t ISR; //!< DMA interrupt status register
        volatile register_t IFCR; //!< DMA interrupt flag clear register

        // Channel array (DMA1 has 7 channels)
        DMAChannel CH[7];
    };

    struct DMA2MemoryMap {
        volatile register_t ISR; //!< DMA interrupt status register
        volatile register_t IFCR; //!< DMA interrupt flag clear register
        // Channel array (DMA2 has 5 channels)
        DMAChannel CH[5];
    };

    // =========================================================================
    // Peripheral Base Addresses
    // =========================================================================

    /** @brief Flash Interface Register*/
    inline const auto FlashInterface = reinterpret_cast<Flash *>(0x40023C00);
    inline const auto DMA1 = reinterpret_cast<DMA1MemoryMap *>(0x4002'6000u);
    inline const auto DMA2 = reinterpret_cast<DMA2MemoryMap *>(0x4002'6400u);
    inline const auto USART1 = reinterpret_cast<USART *>(0x4001'1000u);
    /** @brief General Purpose Timer 2 on the APB1 Bus (TIM2 base address). */
    inline const auto TIMER2 = reinterpret_cast<TIMER *>(0x4000'0000u);

    /** @brief General Purpose Timer 3 on the APB1 Bus (TIM3 base address). */
    inline const auto TIMER3 = reinterpret_cast<TIMER *>(0x4000'0400u);

    /** @brief General Purpose Timer 4 on the APB1 Bus (TIM4 base address). */
    inline const auto TIMER4 = reinterpret_cast<TIMER *>(0x4000'0800u);

    /** @brief Reset and Clock Control module on the AHB Bus (RCC base address). */
    inline const auto RCC1 = reinterpret_cast<RCC *>(0x4002'3800u);

    /** @brief GPIO Port A base address. */
    inline const unsigned int GPIOAdressA = 0x4002'0000;

    /** @brief GPIO Port B base address. */
    inline const unsigned int GPIOAdressB = 0x4002'0400;

    /** @brief GPIO Port C base address. */
    inline const unsigned int GPIOAdressC = (0x4002'0800);
    /** @brief GPIO Port A on the APB2 Bus (GPIOA base address). */
    inline const auto GPIOA = reinterpret_cast<GPIO *>(GPIOAdressA);

    /** @brief GPIO Port B on the APB2 Bus (GPIOB base address). */
    inline const auto GPIOB = reinterpret_cast<GPIO *>(GPIOAdressB);

    /** @brief GPIO Port C on the APB2 Bus (GPIOC base address). */
    inline const auto GPIOC = reinterpret_cast<GPIO *>(GPIOAdressC);

}

#endif // WHEEL2FIRMWARE_MEMORYMAP_H
