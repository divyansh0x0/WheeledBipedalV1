#ifndef WHEEL2FIRMWARE_MEMORYMAP_H
#define WHEEL2FIRMWARE_MEMORYMAP_H

#include <cstdint>

namespace F411::MemoryMap {
    inline constexpr unsigned int CPU_FREQUENCY = 72'000'000;
    using register_t = unsigned int;

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

    enum class APB2Peripheral : unsigned int {
        TIMER1 = 0,
        TIMER11 = 18,
        TIMER10 = 17,
        TIMER9 = 16,
        USART1 = 4,
        USART6 = 5,
        ADC1 = 8,
        SDIO = 11,
        SPI1 = 12,
        SPI4 = 13,
        SYSCFG = 14
    };

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
        Flash(const Flash &) = delete;

        Flash &operator=(const Flash &) = delete;

        Flash(Flash &&) = delete;

        Flash &operator=(Flash &&) = delete;

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
    struct TIMER16 {
        TIMER16(const TIMER16 &) = delete;
        TIMER16 &operator=(const TIMER16 &) = delete;
        TIMER16(TIMER16 &&) = delete;
        TIMER16 &operator=(TIMER16 &&) = delete;

        volatile register_t CR1;
        volatile const register_t reserved1;
        volatile const register_t reserved2;
        volatile register_t DIER;
        volatile register_t SR;
        volatile register_t EGR;
        volatile register_t CCMR1;
        volatile const register_t reserved3;
        volatile register_t CCER;
        volatile register_t CNT;
        volatile register_t PSC;
        volatile register_t ARR;
        volatile const register_t reserved4;
        volatile register_t CCR1;
        volatile register_t CCR2;

        void setFrequency(uint32_t system_clock, uint32_t target_hz, uint32_t resolution = 1000) {
            uint32_t psc_value = (system_clock / (target_hz * resolution)) - 1;
            uint32_t arr_value = resolution - 1;

            this->PSC = psc_value;
            this->ARR = arr_value;
        }

        void enablePWM(TimerChannel channel) {
            switch (channel) {
                case TimerChannel::Channel1:
                    this->CCMR1 &= ~(0b11 << 0);
                    this->CCMR1 &= ~(0b111 << 4);
                    this->CCMR1 |= (0b110 << 4);
                    this->CCMR1 |= (1 << 3);
                    this->CCER |= (1 << 0);
                    break;
                case TimerChannel::Channel2:
                    this->CCMR1 &= ~(0b11 << 8);
                    this->CCMR1 &= ~(0b111 << 12);
                    this->CCMR1 |= (0b110 << 12);
                    this->CCMR1 |= (1 << 11);
                    this->CCER |= (1 << 4);
                    break;
                default:
                    static_assert("Error: Only Ch1 and Ch2 are supported in TIM9 while TIM10, TIM11 have CH1 only");
            }
        }

        void setDutyCycle(TimerChannel channel, unsigned int value) {
            switch (channel) {
                case TimerChannel::Channel1: this->CCR1 = value;
                    break;
                case TimerChannel::Channel2: this->CCR2 = value;
                    break;
                default:
                    static_assert("Error: Only Ch1 and Ch2 are supported in TIM9 while TIM10, TIM11 have CH1 only");
            }
        }

        void start() {
            this->CR1 |= (1 << 0);
        }
    };

    struct TIMER {
        TIMER(const TIMER &) = delete;
        TIMER &operator=(const TIMER &) = delete;
        TIMER(TIMER &&) = delete;
        TIMER &operator=(TIMER &&) = delete;

        volatile register_t CR1;
        volatile register_t CR2;
        volatile register_t SMCR;
        volatile register_t DIER;
        volatile register_t SR;
        volatile register_t EGR;
        volatile register_t CCMR1;
        volatile register_t CCMR2;
        volatile register_t CCER;
        volatile register_t CNT;
        volatile register_t PSC;
        volatile register_t ARR;
        volatile register_t RCR;
        volatile register_t CCR1;
        volatile register_t CCR2;
        volatile register_t CCR3;
        volatile register_t CCR4;

        void setFrequency(uint32_t system_clock, uint32_t target_hz, uint32_t resolution = 1000) {
            uint32_t psc_value = (system_clock / (target_hz * resolution)) - 1;
            uint32_t arr_value = resolution - 1;

            this->PSC = psc_value;
            this->ARR = arr_value;
        }

        void enablePWM(TimerChannel channel) {
            switch (channel) {
                case TimerChannel::Channel1:
                    this->CCMR1 &= ~(0b11 << 0);
                    this->CCMR1 &= ~(0b111 << 4);
                    this->CCMR1 |= (0b110 << 4);
                    this->CCMR1 |= (1 << 3);
                    this->CCER |= (1 << 0);
                    break;
                case TimerChannel::Channel2:
                    this->CCMR1 &= ~(0b11 << 8);
                    this->CCMR1 &= ~(0b111 << 12);
                    this->CCMR1 |= (0b110 << 12);
                    this->CCMR1 |= (1 << 11);
                    this->CCER |= (1 << 4);
                    break;
                case TimerChannel::Channel3:
                    this->CCMR2 &= ~(0b11 << 0);
                    this->CCMR2 &= ~(0b111 << 4);
                    this->CCMR2 |= (0b110 << 4);
                    this->CCMR2 |= (1 << 3);
                    this->CCER |= (1 << 8);
                    break;
                case TimerChannel::Channel4:
                    this->CCMR2 &= ~(0b11 << 8);
                    this->CCMR2 &= ~(0b111 << 12);
                    this->CCMR2 |= (0b110 << 12);
                    this->CCMR2 |= (1 << 11);
                    this->CCER |= (1 << 12);
                    break;
            }
        }

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

        void start() {
            this->CR1 |= (1 << 0);
        }
    };

    static_assert(sizeof(TIMER) == 0x44);

    struct RCC {
        RCC(const RCC &) = delete;
        RCC &operator=(const RCC &) = delete;
        RCC(RCC &&) = delete;
        RCC &operator=(RCC &&) = delete;

        volatile register_t CR;
        volatile register_t PLLCFGR;
        volatile register_t CFGR;
        volatile register_t CIR;
        volatile register_t AHB1RSTR;
        volatile register_t AHB2RSTR;
        volatile register_t __reserved1;
        volatile register_t __reserved2;
        volatile register_t APB1RSTR;
        volatile register_t APB2RSTR;
        volatile register_t __reserved3;
        volatile register_t __reserved4;
        volatile register_t AHB1ENR;
        volatile register_t AHB2ENR;
        volatile register_t __reserved5;
        volatile register_t __reserved6;
        volatile register_t APB1ENR;
        volatile register_t APB2ENR;
        volatile register_t __reserved7;
        volatile register_t __reserved8;
        volatile register_t AHB1LPENR;
        volatile register_t AHB2LPENR;
        volatile register_t __reserved9;
        volatile register_t __reserved10;
        volatile register_t APB1LPENR;
        volatile register_t APB2LPENR;
        volatile register_t __reserved11;
        volatile register_t __reserved12;
        volatile register_t BDCR;
        volatile register_t CSR;
        volatile register_t __reserved13;
        volatile register_t __reserved14;
        volatile register_t SSCGR;
        volatile register_t PLLI2SCFGR;

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

        void enablePLL(PLLSource source, uint32_t m, uint32_t n, uint32_t p, uint32_t q) {
            constexpr unsigned int pll_on_bit = 24;
            constexpr unsigned int pll_ready_bit = 25;

            this->CR &= ~(1 << pll_on_bit);
            waitForBit(&this->CR, pll_ready_bit, false);

            uint32_t pllcfgr = this->PLLCFGR;
            pllcfgr &= ~((0x3F << 0) | (0x1FF << 6) | (0x3 << 16) | (1 << 22) | (0xF << 24));

            uint32_t p_bits = (p / 2) - 1;

            pllcfgr |= (m << 0);
            pllcfgr |= (n << 6);
            pllcfgr |= (p_bits << 16);
            pllcfgr |= (static_cast<uint32_t>(source) << 22);
            pllcfgr |= (q << 24);

            this->PLLCFGR = pllcfgr;
            this->CR |= (1 << pll_on_bit);
            waitForBit(&this->CR, pll_ready_bit, true);
        }

        void setSystemClockSrc(SystemClockSource source) {
            const unsigned int sw_val = static_cast<unsigned int>(source);
            this->CFGR = (this->CFGR & ~0b11) | sw_val;
            while ((this->CFGR & (0b11 << 2)) != (sw_val << 2));
        }

        void setAPB1PreScaler(Prescaler scale) {
            constexpr unsigned int ppre_bit = 10;
            this->CFGR &= ~(0b111 << ppre_bit);
            this->CFGR |= static_cast<unsigned int>(scale) << ppre_bit;
        }

        void setAPB2PreScaler(Prescaler scale) {
            constexpr unsigned int ppre_bit = 13;
            this->CFGR &= ~(0b111 << ppre_bit);
            this->CFGR |= static_cast<unsigned int>(scale) << ppre_bit;
        }

        void enablePeripheral(APB2Peripheral peripheral) {
            this->APB2ENR |= (1 << static_cast<unsigned int>(peripheral));
        }

        void enablePeripheral(APB1Peripheral peripheral) {
            this->APB1ENR |= (1 << static_cast<unsigned int>(peripheral));
        }

        void enablePeripheral(AHB1Peripheral peripheral) {
            this->AHB1ENR |= (1 << static_cast<unsigned int>(peripheral));
        }

        void setAHBPrescaler(AHBPrescaler scale) {
            const unsigned int hpre_bit = 4;
            this->CFGR &= ~(0b1111 << hpre_bit);
            this->CFGR |= static_cast<unsigned int>(scale) << hpre_bit;
        }
    };


    struct GPIORegister {
        GPIORegister(const GPIORegister &) = delete;
        GPIORegister &operator=(const GPIORegister &) = delete;
        GPIORegister(GPIORegister &&) = delete;
        GPIORegister &operator=(GPIORegister &&) = delete;

        volatile register_t MODER;
        volatile register_t OTYPER;
        volatile register_t OSPEEDR;
        volatile register_t PUPDR;
        volatile register_t IDR;
        volatile register_t ODR;
        volatile register_t BSRR;
        volatile register_t LCKR;
        volatile register_t AFR[2];

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
            High_100MHz = 0b11
        };

        enum class Pull : uint32_t {
            None = 0b00,
            Up = 0b01,
            Down = 0b10
        };

        void setPinHigh(unsigned int pin_number) {
            this->BSRR = (1u << pin_number);
        }

        void setPinLow(unsigned int pin_number) {
            this->BSRR = (1u << (pin_number + 16));
        }

        void configurePin(unsigned int pin,
                          Mode mode,
                          Pull pull = Pull::None,
                          OutputSpeed speed = OutputSpeed::Low_2MHz,
                          OutputType type = OutputType::PushPull) {
            const unsigned int shift2 = pin * 2;
            const uint32_t mask2 = 0b11;

            const unsigned int shift1 = pin;
            const uint32_t mask1 = 0b1;

            uint32_t moder = this->MODER;
            moder &= ~(mask2 << shift2);
            moder |= (static_cast<uint32_t>(mode) << shift2);
            this->MODER = moder;

            uint32_t pupdr = this->PUPDR;
            pupdr &= ~(mask2 << shift2);
            pupdr |= (static_cast<uint32_t>(pull) << shift2);
            this->PUPDR = pupdr;

            uint32_t ospeedr = this->OSPEEDR;
            ospeedr &= ~(mask2 << shift2);
            ospeedr |= (static_cast<uint32_t>(speed) << shift2);
            this->OSPEEDR = ospeedr;

            uint32_t otyper = this->OTYPER;
            otyper &= ~(mask1 << shift1);
            otyper |= (static_cast<uint32_t>(type) << shift1);
            this->OTYPER = otyper;
        }
    };

    struct USART {
        USART(const USART &) = delete;
        USART &operator=(const USART &) = delete;
        USART(USART &&) = delete;
        USART &operator=(USART &&) = delete;

        volatile register_t SR;
        volatile register_t DR;
        volatile register_t BRR;
        volatile register_t CR1;
        volatile register_t CR2;
        volatile register_t CR3;
        volatile register_t GTPR;

        void init(BaudRate baud, unsigned int peripheral_clock) {
            const auto baud_val = static_cast<unsigned int>(baud);

            const unsigned int usartdiv = (peripheral_clock * 10) / (16 * baud_val);
            const unsigned int mantissa = usartdiv / 10;
            const unsigned int fraction = ((usartdiv % 10) * 16 + 5) / 10;

            this->BRR = (mantissa << 4) | (fraction & 0x0F);
            this->CR1 = (1 << 13) | (1 << 3) | (1 << 2);
        }

        unsigned char readByte() const {
            while (!(this->SR & (1 << 5)));
            return static_cast<unsigned char>(this->DR & 0xFF);
        }

        void transmit(uint8_t data) {
            while (!(this->SR & (1 << 7)));
            this->DR = data;
        }

        bool hasData() const {
            return (this->SR & (1 << 5));
        }

        uint8_t receive() const {
            return static_cast<uint8_t>(this->DR & 0xFF);
        }
    };

    struct I2C {
        I2C(const I2C &) = delete;

        I2C &operator=(const I2C &) = delete;

        I2C(I2C &&) = delete;

        I2C &operator=(I2C &&) = delete;

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

    struct DMAStream {
        DMAStream(const DMAStream &) = delete;

        DMAStream &operator=(const DMAStream &) = delete;

        DMAStream(DMAStream &&) = delete;

        DMAStream &operator=(DMAStream &&) = delete;

        volatile register_t CR;
        volatile register_t NDTR;
        volatile register_t PAR;
        volatile register_t M0AR;
        volatile register_t M1AR;
        volatile register_t FCR;

        enum class Channel : register_t {
            CH0 = 0,
            CH1 = 1,
            CH2 = 2,
            CH3 = 3,
            CH4 = 4,
            CH5 = 5,
            CH6 = 6,
            CH7 = 7,
        };

        enum class DMAPriorityLevel : register_t {
            LOW = 0b00,
            MEDIUM = 0b01,
            HIGH = 0b10,
            VERY_HIGH = 0b11
        };

        enum class DMAMemorySize : register_t {
            BYTE = 0b00,
            HALF_WORD = 0b01,
            WORD = 0b10,
        };

        enum class TransferDirection : register_t {
            PERIPHERAL_TO_MEMORY = 0b00,
            MEMORY_TO_PERIPHERAL = 0b01,
            MEMORY_TO_MEMORY = 0b10
        };

        bool isEnabled() const {
            return CR & 0b1;
        }

        void enableCircularMode(bool enabled) {
            if (enabled) {
                CR |= 0b1 << 8;
            } else {
                CR &= ~(0b1 << 10);
            }
        }

        void setEnabled(bool state) {
            if (state) CR |= 1u;
            else CR &= ~1u;
        }

        void enableMemoryIncrementMode(bool enabled) {
            if (enabled) {
                CR |= 0b1 << 10;
            } else {
                CR &= ~(0b1 << 10);
            }
        }

        void enablePeripheralIncrementMode(bool enabled) {
            if (enabled) {
                CR |= 0b1 << 9;
            } else {
                CR &= ~(0b1 << 9);
            }
        }


        void setChannel(Channel channel) {
            constexpr int bit_start = 25;
            CR &= ~(0b111 << bit_start);
            CR |= (static_cast<register_t>(channel) << bit_start);
        }

        void setPriorityLevel(DMAPriorityLevel level) {
            constexpr int bit_start = 16;
            CR &= ~(0b11 << bit_start);
            CR |= (static_cast<register_t>(level) << bit_start);
        }

        void setSize(DMAMemorySize memory_size, DMAMemorySize peripheral_size) {
            constexpr int m_bit_start = 13;
            CR &= ~(0b11 << m_bit_start);
            CR |= (static_cast<register_t>(memory_size) << m_bit_start);

            constexpr int p_bit_start = 11;
            CR &= ~(0b11 << p_bit_start);
            CR |= (static_cast<register_t>(peripheral_size) << p_bit_start);
        }

        void setDataTransferMode(const TransferDirection dir) {
            constexpr unsigned int bit_start = 6;
            CR &= ~(0b11 << bit_start);
            CR |= (static_cast<register_t>(dir) << bit_start);
        }

        void enableTransferCompleteInterrupt(bool enabled) {
            constexpr unsigned int bit_start = 4;
            if (enabled) CR |= (1u << bit_start);
            else CR &= ~(1u << bit_start);
        }

        void setPeripheralAddress(const volatile void *peripheral) {
            PAR = reinterpret_cast<register_t>(peripheral);
        }

        void setMemoryAddress(const volatile void *memory) {
            M0AR = reinterpret_cast<register_t>(memory);
        }

        void setDataLength(uint32_t length) {
            NDTR = length;
        }

        enum class FIFOStorageWord {
            OneWord,
            TwoWord,
            ThreeWord,
            FourWord,
        };

        void enableFIFO(bool enabled, FIFOStorageWord level) {
            if (enabled) {
                FCR &= ~0xF;
                FCR |= static_cast<uint32_t>(level);
                FCR |= 1 << 2;
            } else {
                FCR &= ~(1 << 2);
            }
        }

        void clearInterruptFlags() {
            uint32_t addr = reinterpret_cast<uint32_t>(this);
            uint32_t stream_idx = ((addr & 0xFF) - 0x10) / 0x18; 
            
            volatile register_t* lifcr = reinterpret_cast<volatile register_t*>( (addr & ~0x3FF) + 0x08 );
            volatile register_t* hifcr = reinterpret_cast<volatile register_t*>( (addr & ~0x3FF) + 0x0C );
            
            uint32_t offsets[4] = {0, 6, 16, 22};
            uint32_t offset = offsets[stream_idx % 4];
            uint32_t mask = 0x3D << offset;
            
            if (stream_idx < 4) {
                *lifcr = mask;
            } else {
                *hifcr = mask;
            }
        }
    };

    struct DMAMemoryMap {
        volatile register_t LISR;
        volatile register_t HISR;
        volatile register_t LIFCR;
        volatile register_t HIFCR;
        DMAStream STREAMS[8];
    };

    struct RTCReg {
        volatile register_t TR;
        volatile register_t DR;
        volatile register_t CR;
        volatile register_t ISR;
        volatile register_t PRER;
        volatile register_t WUTR;
        volatile register_t CALIBR;
        volatile register_t ALRMAR;
        volatile register_t ALRMBR;
        volatile register_t WPR;
        volatile register_t SSR;
        volatile register_t SHIFTR;
        volatile register_t TSTR;
        volatile register_t TSSSR;
        volatile register_t CALR;
        volatile register_t TAFCR;
        volatile register_t ALRMASSR;
        volatile register_t ALRMBSSR;
        volatile register_t BKP0R;
        volatile register_t BKP19R;
    };

    struct ADCReg {
        ADCReg(const ADCReg &) = delete;
        ADCReg &operator=(const ADCReg &) = delete;
        ADCReg(ADCReg &&) = delete;
        ADCReg &operator=(ADCReg &&) = delete;

        volatile register_t SR;
        volatile register_t CR1;
        volatile register_t CR2;
        volatile register_t SMPR1;
        volatile register_t SMPR2;
        volatile register_t JOFR1;
        volatile register_t JOFR2;
        volatile register_t JOFR3;
        volatile register_t JOFR4;
        volatile register_t HTR;
        volatile register_t LTR;
        volatile register_t SQR1;
        volatile register_t SQR2;
        volatile register_t SQR3;
        volatile register_t JSQR;
        volatile register_t JDR1;
        volatile register_t JDR2;
        volatile register_t JDR3;
        volatile register_t JDR4;
        volatile register_t DR;
        volatile register_t CCR;
    };

    struct SysTickReg {
        volatile register_t CTRL;
        volatile register_t LOAD;
        volatile register_t VAL;
        volatile register_t CALIB;
    };
    struct SYSCFGReg {
        volatile uint32_t MEMRMP;
        volatile uint32_t PMC;
        volatile uint32_t EXTICR[4];
        uint32_t RESERVED[2];
        volatile uint32_t CMPCR;
    };
    inline auto SYSCFG = reinterpret_cast<SYSCFGReg *>(0x40013800);

    inline const auto SysTick = reinterpret_cast<SysTickReg *>(0xE000'E010);
    inline const auto FlashInterface = reinterpret_cast<Flash *>(0x4002'3C00);
    inline const auto DMA1 = reinterpret_cast<DMAMemoryMap *>(0x4002'6000u);
    inline const auto DMA2 = reinterpret_cast<DMAMemoryMap *>(0x4002'6400u);
    inline const auto USART1 = reinterpret_cast<USART *>(0x4001'1000u);
    inline const auto TIMER2 = reinterpret_cast<TIMER *>(0x4000'0000u);
    inline const auto TIMER3 = reinterpret_cast<TIMER *>(0x4000'0400u);
    inline const auto TIMER4 = reinterpret_cast<TIMER *>(0x4000'0800u);
    inline const auto TIMER5 = reinterpret_cast<TIMER *>(0x4000'0C00u);
    inline const auto TIMER9 = reinterpret_cast<TIMER16*>(0x4001'4800u);
    inline const auto TIMER10 = reinterpret_cast<TIMER16 *>(0x4001'4400u);
    inline const auto TIMER11 = reinterpret_cast<TIMER16 *>(0x4001'4000u);
    inline const auto RCC1 = reinterpret_cast<RCC *>(0x4002'3800u);

    inline const unsigned int GPIOAdressA = 0x4002'0000;
    inline const unsigned int GPIOAdressB = 0x4002'0400;
    inline const unsigned int GPIOAdressC = (0x4002'0800);
    inline const auto GPIOA = reinterpret_cast<GPIORegister *>(GPIOAdressA);
    inline const auto GPIOB = reinterpret_cast<GPIORegister *>(GPIOAdressB);
    inline const auto GPIOC = reinterpret_cast<GPIORegister *>(GPIOAdressC);

    inline const auto ADC = reinterpret_cast<ADCReg *>(0x4001'2000);
}

#endif // WHEEL2FIRMWARE_MEMORYMAP_H
