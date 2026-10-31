#ifndef BIPEDALV1_I2C_H
#define BIPEDALV1_I2C_H
#include "MemoryMap.h"
#include "Clock.h"
#include "GPIO.h"
#include "Interrupt.h"

namespace F411 {
    using I2CWriteCallback = void(*)(void* ctx);
    using I2CReadCallback = void(*)(void* ctx);

    template<unsigned int addr>
    class I2C {
        static inline I2CWriteCallback m_write_callback;
        static inline I2CReadCallback m_read_callback;
        static inline void* m_callback_ctx;
        enum I2CFlags : uint16_t {
            START_BIT_GENERATED = (1 << 0),
            ADDRESS_SENT = (1 << 1),
            BYTE_TRANSFER_FINISHED = (1 << 2),
            HEADER_SENT = (1 << 3),
            STOP_DETECTED = (1 << 4),
            RXNE = (1 << 6),
            TRANSFER_REGISTER_EMPTY = (1 << 7),
            BUS_ERROR = (1 << 8),
            ARBITRATION_LOST = (1 << 9),
            ACKNOWLEDGE_FAILURE = (1 << 10),
            OVR = (1 << 11),
            PEC_ERROR = (1 << 12),
            TIMEOUT = (1 << 14)
        };

        static bool waitEvent(I2CFlags flag) {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            uint32_t timeout_ms = 1;

            const uint32_t start = Clock::millis();
            while (!(REG->SR1 & flag)) {
                if (REG->SR1 & (I2CFlags::ACKNOWLEDGE_FAILURE | I2CFlags::BUS_ERROR | I2CFlags::ARBITRATION_LOST)) {
                    REG->SR1 &= ~(I2CFlags::ACKNOWLEDGE_FAILURE | I2CFlags::BUS_ERROR | I2CFlags::ARBITRATION_LOST);
                    REG->CR1 |= (1 << 15);
                    REG->CR1 &= ~(1 << 15);
                    enable(true);
                    return false;
                }

                if (Clock::millis() - start > timeout_ms) {
                    REG->CR1 |= (1 << 15);
                    REG->CR1 &= ~(1 << 15);
                    enable(true);
                    return false;
                }
            }
            return true;
        }

        static inline void clearAddress() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            (void) REG->SR1;
            (void) REG->SR2;
        }

    public:
        static void enable(bool enableDMA = true) {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            REG->CR1 &= ~1u;
            if (enableDMA) {
                REG->CR2 |= 1 << 11;
            }
            const auto apb1_freq_hz = Clock::getAPB1Clock();
            const auto bus_freq_mhz = apb1_freq_hz / 1'000'000;
            REG->CR2 = (REG->CR2 & ~0x3Fu) | (bus_freq_mhz & 0x3F);

            const uint32_t ccr_val = apb1_freq_hz / (25 * 400'000);
            REG->CCR = (REG->CCR & ~0xCFFFu) | (1 << 15) | (1 << 14) | (ccr_val << 0);
            REG->TRISE = (REG->TRISE & ~0x3Fu) | ((300 * bus_freq_mhz) / 1000 + 1);
            REG->CR1 |= 1u;
        }

        template<typename SclPin, typename SdaPin, typename SclPeriph, typename SdaPeriph>
        static void recoverBus() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            REG->CR1 &= ~1u;

            SclPin::enableOutputMode();
            SdaPin::enableOutputMode();

            const auto scl_port = reinterpret_cast<volatile MemoryMap::GPIORegister *>(SclPin::port_address);
            scl_port->OTYPER |= (1 << SclPin::pin_index);
            const auto sda_port = reinterpret_cast<volatile MemoryMap::GPIORegister *>(SdaPin::port_address);
            sda_port->OTYPER |= (1 << SdaPin::pin_index);

            SclPin::set(HIGH);
            SdaPin::set(HIGH);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");

            for (int i = 0; i < 9; ++i) {
                if (SdaPin::getInputState() == HIGH) {
                    break;
                }
                SclPin::set(LOW);
                for (int j = 0; j < 1000; ++j) asm volatile("nop");
                SclPin::set(HIGH);
                for (int j = 0; j < 1000; ++j) asm volatile("nop");
            }

            SdaPin::set(LOW);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");
            SclPin::set(HIGH);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");
            SdaPin::set(HIGH);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");

            SclPin::template enableAlternateFunction<SclPeriph>();
            SdaPin::template enableAlternateFunction<SdaPeriph>();
            enable(true);
        }

        static bool isBusBusy() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            return (REG->SR2 & (1 << 1)) != 0;
        }
        static void setCallbacks(I2CReadCallback read_finished, I2CWriteCallback write_finished, void* ctx){
            m_read_callback = read_finished;
            m_write_callback = write_finished;
            m_callback_ctx = ctx;
        }
        static bool waitFreeBus() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            const uint32_t start = Clock::micros();
            while ((REG->SR2 & (1 << 1)) || (REG->CR1 & (1 << 9))) {
                if (Clock::micros() - start > 500) {
                    REG->CR1 |= (1 << 15);
                    REG->CR1 &= ~(1 << 15);
                    enable(true);
                    return false;
                }
            }
            return true;
        }

        static bool probe(uint8_t i2c_addr) {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            if (!waitFreeBus()) return false;
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) return false;
            REG->DR = i2c_addr << 1;
            bool ack = waitEvent(I2CFlags::ADDRESS_SENT);
            clearAddress();
            REG->CR1 |= 1 << 9;
            return ack;
        }

        static bool writeRegister(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *data, const uint32_t length,
                                  bool use_dma = false) {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);

            if (!waitFreeBus()) return false;

            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) return false;
            REG->DR = i2c_addr << 1;
            if (!waitEvent(I2CFlags::ADDRESS_SENT)) return false;
            clearAddress();

            REG->DR = reg_addr;
            if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;

            if (length == 0) {
                if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;
                REG->CR1 |= 1 << 9;
            } else if (use_dma) {
                MemoryMap::DMAStream *dma_stream = getDMAStreamWrite();

                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH1);
                dma_stream->setEnabled(false);
                dma_stream->setDataTransferMode(MemoryMap::DMAStream::TransferDirection::MEMORY_TO_PERIPHERAL);
                while (dma_stream->isEnabled()) {
                };
                dma_stream->setPeripheralAddress(&REG->DR);
                dma_stream->setMemoryAddress(data);
                dma_stream->setDataLength(length);
                dma_stream->enableMemoryIncrementMode(true);
                dma_stream->enableTransferCompleteInterrupt(true);
                REG->CR2 |= (1 << 11);

                dma_stream->clearInterruptFlags();
                dma_stream->setEnabled(true);
            } else {
                for (uint32_t i = 0; i < length; i++) {
                    REG->DR = data[i];
                    if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;
                }
                if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;
                REG->CR1 |= 1 << 9;
            }

            return true;
        }

        static void finishDMAWrite() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED);
            REG->CR1 |= (1 << 9);
            REG->CR2 &= ~(1 << 11);
        }

        static void finishDMARead() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            REG->CR1 |= (1 << 9);
            REG->CR2 &= ~(1 << 11 | 1 << 12);
            REG->CR1 |= (1 << 10);
        }

        static constexpr MemoryMap::DMAStream *getDMAStreamRead() {
            MemoryMap::DMAStream *dma_stream = nullptr;
            unsigned int stream_id;
            if constexpr (addr == 0x4000'5400) {
                stream_id = MemoryMap::DMA1->STREAMS[0].isEnabled() ? 5 : 0;
                dma_stream = &MemoryMap::DMA1->STREAMS[stream_id];
                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH1);
            }

            if constexpr (addr == 0x4000'5800) {
                stream_id = MemoryMap::DMA1->STREAMS[2].isEnabled() ? 3 : 2;
                dma_stream = &MemoryMap::DMA1->STREAMS[stream_id];
                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH7);
            }
            if constexpr (addr == 0x4000'5C00) {
                stream_id = 2;
                dma_stream = &MemoryMap::DMA1->STREAMS[stream_id];
                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH3);
            }

            if (dma_stream && dma_stream->isEnabled()) {
                return nullptr;
            }
            if (dma_stream) {
                InterruptManager::attachDMAInterrupt(static_cast<InterruptManager::DMAStream>(stream_id), [] {
                    finishDMARead();
                    m_read_callback(m_callback_ctx);
                });
            }
            return dma_stream;
        }

        static constexpr MemoryMap::DMAStream *getDMAStreamWrite() {
            MemoryMap::DMAStream *dma_stream = nullptr;
            unsigned int stream_id;
            if constexpr (addr == 0x4000'5400) {
                stream_id = MemoryMap::DMA1->STREAMS[6].isEnabled() ? 7 : 6;
                dma_stream = &MemoryMap::DMA1->STREAMS[stream_id];
                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH1);
            }

            if constexpr (addr == 0x4000'5800) {
                stream_id = 7;
                dma_stream = &MemoryMap::DMA1->STREAMS[stream_id];
                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH7);
            }
            if constexpr (addr == 0x4000'5C00) {
                stream_id = 4;
                dma_stream = &MemoryMap::DMA1->STREAMS[stream_id];
                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH3);
            }

            if (dma_stream && dma_stream->isEnabled()) {
                return nullptr;
            }
            if (dma_stream) {
                InterruptManager::attachDMAInterrupt(static_cast<InterruptManager::DMAStream>(stream_id), [] {
                    finishDMAWrite();
                    m_write_callback(m_callback_ctx);

                });
            }
            return dma_stream;
        }

        static bool readRegister(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, const uint32_t length,
                                 bool use_dma = false) {
            if (length == 0) return true;
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);

            if (!waitFreeBus()) return false;
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) return false;
            REG->DR = i2c_addr << 1;
            if (!waitEvent(I2CFlags::ADDRESS_SENT)) return false;
            clearAddress();

            REG->DR = reg_addr;
            if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;

            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED))return false;
            REG->DR = (i2c_addr << 1) | 1u;
            if (!waitEvent(I2CFlags::ADDRESS_SENT))return false;
            if (use_dma) {
                MemoryMap::DMAStream *dma_stream = getDMAStreamRead();
                if (dma_stream == nullptr) {
                    REG->CR1 |= (1 << 9);
                    return false;
                }
                dma_stream->setEnabled(false);
                while (dma_stream->isEnabled()) {
                };
                dma_stream->setPeripheralAddress(&REG->DR);
                dma_stream->setMemoryAddress(data);
                dma_stream->setDataLength(length);
                dma_stream->enableMemoryIncrementMode(true);
                dma_stream->enableTransferCompleteInterrupt(true);
                dma_stream->setDataTransferMode(MemoryMap::DMAStream::TransferDirection::PERIPHERAL_TO_MEMORY);
                REG->CR1 |= (1 << 10);
                REG->CR2 |= (1 << 11);
                REG->CR2 |= (1 << 12);

                dma_stream->clearInterruptFlags();
                dma_stream->setEnabled(true);
                
                clearAddress();
            } else {
                if (length == 1) {
                    REG->CR1 &= ~(1 << 10);
                    clearAddress();

                    REG->CR1 |= (1 << 9);
                    if (!waitEvent(I2CFlags::RXNE)) return false;
                    *data = REG->DR;
                } else if (length == 2) {
                    REG->CR1 |= (1 << 11);
                    REG->CR1 &= ~(1 << 10);
                    clearAddress();

                    if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;

                    REG->CR1 |= (1 << 9);

                    *data++ = REG->DR;
                    *data = REG->DR;
                } else {
                    clearAddress();

                    uint32_t remaining = length;
                    while (remaining > 0) {
                        if (remaining == 3) {
                            if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;

                            REG->CR1 &= ~(1 << 10);
                            *data++ = REG->DR;
                            remaining--;
                        } else if (remaining == 2) {
                            if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;

                            REG->CR1 |= (1 << 9);

                            *data++ = REG->DR;
                            remaining--;
                            *data++ = REG->DR;
                            remaining--;
                        } else {
                            if (!waitEvent(I2CFlags::RXNE)) return false;
                            *data++ = REG->DR;
                            remaining--;
                        }
                    }
                }
                REG->CR1 &= ~(1 << 11);
                REG->CR1 |= (1 << 10);
            }

            return true;
        }
    };

    using I2C1 = I2C<0x4000'5400>;
    using I2C2 = I2C<0x4000'5800>;
    using I2C3 = I2C<0x4000'5C00>;
}
#endif //BIPEDALV1_I2C_H
