//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_I2C_H
#define BIPEDALV1_I2C_H
#include "MemoryMap.h"
#include "Clock.h"
#include "GPIO.h"
#include "Interrupt.h"

namespace STM32F411 {
    using I2CWriteCallback = void(*)(void* ctx);
    using I2CReadCallback = void(*)(void* ctx);

    template<unsigned int addr>
    class I2C {
        static inline I2CWriteCallback m_write_callback;
        static inline I2CReadCallback m_read_callback;
        static inline void* m_callback_ctx;
        enum I2CFlags : uint16_t {
            START_BIT_GENERATED = (1 << 0), // Start Bit generated
            ADDRESS_SENT = (1 << 1), // Address Sent / Matched
            BYTE_TRANSFER_FINISHED = (1 << 2), // Byte Transfer Finished
            HEADER_SENT = (1 << 3), // 10-bit Header Sent
            STOP_DETECTED = (1 << 4), // Stop Detection (Slave mode)
            RXNE = (1 << 6), // Data Register Not Empty (Receive)
            TRANSFER_REGISTER_EMPTY = (1 << 7), // Data Register Empty (Transmit)
            BUS_ERROR = (1 << 8), // Bus Error (misplaced Start/Stop)
            ARBITRATION_LOST = (1 << 9), // Arbitration Lost
            ACKNOWLEDGE_FAILURE = (1 << 10), // Acknowledge Failure (Slave sent NACK)
            OVR = (1 << 11), // Overrun / Underrun
            PEC_ERROR = (1 << 12), // PEC Error
            TIMEOUT = (1 << 14) // Timeout / Tlow Error
        };

        static bool waitEvent(I2CFlags flag) {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            uint32_t timeout_ms = 1;

            const uint32_t start = Clock::millis();
            while (!(REG->SR1 & flag)) {
                // If a NACK is received, or a bus error happens, bail out instantly
                if (REG->SR1 & (I2CFlags::ACKNOWLEDGE_FAILURE | I2CFlags::BUS_ERROR | I2CFlags::ARBITRATION_LOST)) {
                    REG->SR1 &= ~(I2CFlags::ACKNOWLEDGE_FAILURE | I2CFlags::BUS_ERROR | I2CFlags::ARBITRATION_LOST);

                    // Reset the peripheral on error to avoid bus lockup
                    REG->CR1 |= (1 << 15); // Set SWRST
                    REG->CR1 &= ~(1 << 15); // Clear SWRST
                    enable(); // Re-initialize I2C

                    return false;
                }

                if (Clock::millis() - start > timeout_ms) {
                    // Reset the peripheral on timeout to avoid bus lockup
                    REG->CR1 |= (1 << 15); // Set SWRST
                    REG->CR1 &= ~(1 << 15); // Clear SWRST
                    enable(); // Re-initialize I2C

                    return false; // Software timeout
                }
            }
            return true;
        }

        static inline void clearAddress() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            (void) REG->SR1;
            (void) REG->SR2; // Clear ADDR
        }

    public:
        static void enable(bool enableDMA = false) {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            REG->CR1 &= ~1u; // set EN to false
            if (enableDMA) {
                REG->CR2 |= 1 << 11;
            }
            const auto apb1_freq_hz = Clock::getAPB1Clock();
            const auto bus_freq_mhz = apb1_freq_hz / 1'000'000;
            REG->CR2 = (REG->CR2 & ~0x3Fu) | (bus_freq_mhz & 0x3F);
            // SCL Clock speed. For slow mode (upto 100khz):
            // The high time and low time are equal for the clock
            // T_high = CCR * T_PCLK1, and T_high = T_low
            // So for slow mode, SCL = f_PCLK1/(2*CCR)

            // In fast mode the I2C forces the line to spend more time low than high due to capacitance in the wires
            // There are two options for the ratio of low to high time period
            // 1.  2:1, 33% HIGH and 66% low set by setting bit 14 (DUTY) to 0. CCR = f_pclk/(3 * f_scl)
            // 2. 16/9, which is set by setting bit 14 to 1, CCR = f_pclk(25*f_scl)

            // Using FM mode (bit 15) with 16/9 duty ratio and setting SCL freq to 400khz giving CCR (starts at bit 0) = 4

            const uint32_t ccr_val = apb1_freq_hz / (25 * 400'000);
            REG->CCR = (REG->CCR & ~0xCFFFu) | (1 << 15) | (1 << 14) | (ccr_val << 0);

            REG->TRISE = (REG->TRISE & ~0x3Fu) | ((300 * bus_freq_mhz) / 1000 + 1);
            // set Trise = MAX_RISE_TIME * BUS_FREQ + 1

            REG->CR1 |= 1u; // set EN to true
        }

        /**
         * Write a single raw byte to a device that has no register address
         * (e.g. PCA9548A multiplexer).
         * Sends: START → i2c_addr+W → byte → STOP
         */
        template<typename SclPin, typename SdaPin, typename SclPeriph, typename SdaPeriph>
        static void recoverBus() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            // 1. Disable I2C
            REG->CR1 &= ~1u;

            // 2. Configure SCL and SDA as Output
            SclPin::enableOutputMode();
            SdaPin::enableOutputMode();

            // Set as Open-Drain using the exposed port_address and pin_index
            const auto scl_port = reinterpret_cast<volatile MemoryMap::GPIORegister *>(SclPin::port_address);
            scl_port->OTYPER |= (1 << SclPin::pin_index);
            const auto sda_port = reinterpret_cast<volatile MemoryMap::GPIORegister *>(SdaPin::port_address);
            sda_port->OTYPER |= (1 << SdaPin::pin_index);

            // Set both HIGH
            SclPin::set(HIGH);
            SdaPin::set(HIGH);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");

            // 3. Toggle SCL up to 9 times to clock out stuck slaves
            for (int i = 0; i < 9; ++i) {
                if (SdaPin::getStatus() == HIGH) {
                    break;
                }
                SclPin::set(LOW);
                for (int j = 0; j < 1000; ++j) asm volatile("nop");
                SclPin::set(HIGH);
                for (int j = 0; j < 1000; ++j) asm volatile("nop");
            }

            // 4. Generate STOP condition manually
            SdaPin::set(LOW);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");
            SclPin::set(HIGH);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");
            SdaPin::set(HIGH);
            for (int j = 0; j < 1000; ++j) asm volatile("nop");

            // 5. Reconfigure as Alternate Function
            SclPin::template enableAlternateFunction<SclPeriph>();
            SdaPin::template enableAlternateFunction<SdaPeriph>();

            // 6. Re-enable I2C
            enable();
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
            while (REG->SR2 & (1 << 1)) {
                // A normal STOP bit takes ~2.5us at 400kHz.
                // If it's busy for > 500us, the bus is locked up (slave holding SDA low).
                if (Clock::micros() - start > 500) {
                    REG->CR1 |= (1 << 15);
                    REG->CR1 &= ~(1 << 15);
                    enable();
                    return false;
                }
            }
            return true;
        }

        static bool writeByte(uint8_t i2c_addr, uint8_t byte) {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);

            if (!waitFreeBus()) return false;

            // START
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) {
                return false;
            }

            // Address + Write
            REG->DR = i2c_addr << 1;
            if (!waitEvent(I2CFlags::ADDRESS_SENT)) {
                return false;
            }
            clearAddress();

            // Data byte
            REG->DR = byte;
            if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) {
                return false;
            }
            if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) {
                return false;
            }

            // STOP
            REG->CR1 |= 1 << 9;
            return true;
        }

        static bool writeRegister(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *data, const uint32_t length,
                                  bool use_dma = false) {
            if (length == 0) return true;
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);


            if (!waitFreeBus()) return false;

            // START Transaction
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) return false;
            // SEND I2C address of receiver
            REG->DR = i2c_addr << 1;
            if (!waitEvent(I2CFlags::ADDRESS_SENT)) return false;
            clearAddress();

            REG->DR = reg_addr;
            if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;

            if (use_dma) {
                MemoryMap::DMAStream *dma_stream = getDMAStreamWrite();

                dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH1);
                dma_stream->setEnabled(false);
                while (dma_stream->isEnabled()) {
                };
                dma_stream->setPeripheralAddress(&REG->DR);
                dma_stream->setMemoryAddress(data);
                dma_stream->setDataLength(length);
                dma_stream->enableMemoryIncrementMode(true);
                dma_stream->enableTransferCompleteInterrupt(true);
                REG->CR2 |= (1 << 11); // Set DMAEN
                dma_stream->setEnabled(true); // Enable stream
            } else {
                // Send data
                for (uint32_t i = 0; i < length; i++) {
                    REG->DR = data[i];
                    if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;
                }
                if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;

                //STOP
                REG->CR1 |= 1 << 9;
            }

            return true;
        }

        static void finishDMAWrite() {
            // Must wait for the final bits to physically exit the shift register
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED);
            REG->CR1 |= (1 << 9); // Generate STOP condition
            REG->CR2 &= ~(1 << 11); // Clear DMAEN bit
        }

        static void finishDMARead() {
            const volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
            REG->CR1 |= (1 << 9); // Generate STOP condition
            REG->CR2 &= ~(1 << 11 | 1 << 12); // Clear DMAEN and LAST bits
            REG->CR1 |= (1 << 10); // Re-enable ACK for the next transaction
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
                InterruptManager::attachDMAInterrupt(static_cast<InterruptManager::Stream>(stream_id), [] {
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
                InterruptManager::attachDMAInterrupt(static_cast<InterruptManager::Stream>(stream_id), [] {
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
            //Start
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) return false;
            // send device address with write access
            REG->DR = i2c_addr << 1;
            if (!waitEvent(I2CFlags::ADDRESS_SENT)) return false;
            clearAddress();

            REG->DR = reg_addr;
            if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;

            //repeated start and read
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED))return false;
            REG->DR = (i2c_addr << 1) | 1u; // Device Addr (For read access we set 0th bit to 1)
            if (!waitEvent(I2CFlags::ADDRESS_SENT))return false;
            if (use_dma) {
                MemoryMap::DMAStream *dma_stream = getDMAStreamRead();
                if (dma_stream == nullptr) {
                    // Previous DMA still in progress — abort this read gracefully
                    REG->CR1 |= (1 << 9); // Generate STOP
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

                REG->CR1 |= (1 << 10); // Enable ACK so all bytes except last are ACKed
                REG->CR2 |= (1 << 11); // DMAEN (Enable DMA requests)
                REG->CR2 |= (1 << 12); // Set LAST bit to send NACK on final byte
                clearAddress(); // Clear ADDR flag to start receiving
                dma_stream->setEnabled(true);
            } else {
                if (length == 1) {
                    // Clear ACK
                    REG->CR1 &= ~(1 << 10);
                    clearAddress();

                    // Stop
                    REG->CR1 |= (1 << 9);
                    if (!waitEvent(I2CFlags::RXNE)) return false;
                    *data = REG->DR;
                } else if (length == 2) {
                    // STM32F411 Quirk: For exactly 2 bytes, we must set POS and clear ACK
                    // BEFORE clearing the ADDR flag (EV6).
                    REG->CR1 |= (1 << 11); // Set POS bit
                    REG->CR1 &= ~(1 << 10); // Clear ACK bit
                    clearAddress(); // Now clear ADDR

                    // Wait for BTF (EV8_2). This means BOTH bytes are fully received.
                    if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;

                    REG->CR1 |= (1 << 9); // Generate STOP

                    *data++ = REG->DR; // Read the first byte
                    *data = REG->DR; // Read the second byte
                } else {
                    // For 3 or more bytes
                    clearAddress(); // Clear ADDR immediately

                    uint32_t remaining = length;
                    while (remaining > 0) {
                        if (remaining == 3) {
                            // We are 3 bytes away from the end.
                            // Wait for BTF so we know the last 2 bytes are securely buffering.
                            if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;

                            REG->CR1 &= ~(1 << 10); // Clear ACK to NACK the final byte
                            *data++ = REG->DR; // Read Byte N-2
                            remaining--;
                        } else if (remaining == 2) {
                            // Wait for BTF again for the final two bytes
                            if (!waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED)) return false;

                            REG->CR1 |= (1 << 9); // Generate STOP

                            *data++ = REG->DR; // Read Byte N-1
                            remaining--;
                            *data++ = REG->DR; // Read Byte N
                            remaining--;
                        } else {
                            // Standard read loop for the early bytes
                            if (!waitEvent(I2CFlags::RXNE)) return false; // Wait EV7
                            *data++ = REG->DR;
                            remaining--;
                        }
                    }
                }
                // Cleanup: Reset the hardware state for the next transaction
                REG->CR1 &= ~(1 << 11); // Clear POS
                REG->CR1 |= (1 << 10); // Re-enable ACK
            }


            return true;
        }
    };

    using I2C1 = I2C<0x4000'5400>;
    using I2C2 = I2C<0x4000'5800>;
    using I2C3 = I2C<0x4000'5C00>;
}
#endif //BIPEDALV1_I2C_H
