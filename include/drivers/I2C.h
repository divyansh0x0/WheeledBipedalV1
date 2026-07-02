//
// Created by divyansh on 6/29/26.
//

#ifndef BIPEDALV1_I2C_H
#define BIPEDALV1_I2C_H
#include "MemoryMap.h"
#include "Clock.h"

namespace STM32F411 {
    template<unsigned int addr>
    class I2C {
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
            uint32_t timeout_ms = 1;

            const uint32_t start = Clock::millis();
            while (!(REG->SR1 & flag)) {
                // If a NACK is received, or a bus error happens, bail out instantly
                if (REG->SR1 & (I2CFlags::ACKNOWLEDGE_FAILURE | I2CFlags::BUS_ERROR | I2CFlags::ARBITRATION_LOST)) {
                    REG->SR1 &= ~(I2CFlags::ACKNOWLEDGE_FAILURE | I2CFlags::BUS_ERROR | I2CFlags::ARBITRATION_LOST);
                    return false;
                }

                if (Clock::millis() - start > timeout_ms) {
                    return false; // Software timeout
                }
            }
            return true;
        }

        static inline void clearAddress() {
            (void) REG->SR1;
            (void) REG->SR2; // Clear ADDR
        }

    public:
        static constexpr volatile auto REG = reinterpret_cast<volatile MemoryMap::I2C *>(addr);
        static void enable(bool enableDMA = false) {
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

            REG->TRISE = (REG->TRISE & ~0x3Fu) | (300 * bus_freq_mhz) / 1000 + 1; // set Trise = MAX_RISE_TIME * BUS_FREQ + 1

            REG->CR1 |= 1u; // set EN to true
        }

        static bool writeRegister(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *data, const uint32_t length,
                                  MemoryMap::DMAStream *dma_stream = nullptr) {
            if (length == 0) return true;
            if (dma_stream) {
                dma_stream->setEnabled(false);
                while (dma_stream->isEnabled()) {
                };
                dma_stream->setPeripheralAddress(&REG->DR);
                dma_stream->setMemoryAddress(data);
                dma_stream->setDataLength(length);
                dma_stream->enableMemoryIncrement(true);
                dma_stream->enableTransferCompleteInterrupt(true);
            }

            // START Transaction
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) return false;
            // SEND I2C address of receiver
            REG->DR = i2c_addr << 1;
            if (!waitEvent(I2CFlags::ADDRESS_SENT)) return false;
            clearAddress();

            REG->DR = reg_addr;
            if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;

            if (dma_stream) {
                // 3. Trigger DMA
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
            waitEvent(I2CFlags::BYTE_TRANSFER_FINISHED);
            REG->CR1 |= (1 << 9); // Generate STOP condition
            REG->CR2 &= ~(1 << 11); // Clear DMAEN bit
        }

        static void finishDMARead() {
            REG->CR1 |= (1 << 9); // Generate STOP condition
            REG->CR2 &= ~(1 << 11 | 1 << 12); // Clear DMAEN and LAST bits
            REG->CR1 |= (1 << 10); // Re-enable ACK for the next transaction
        }

        static bool readRegister(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, const uint32_t length,
                                 MemoryMap::DMAStream *dma_stream = nullptr) {
            if (length == 0) return true;
            if (dma_stream) {
                dma_stream->setEnabled(false);
                while (dma_stream->isEnabled()) {
                };
                dma_stream->setPeripheralAddress(&REG->DR);
                dma_stream->setMemoryAddress(data);
                dma_stream->setDataLength(length);
                dma_stream->enableMemoryIncrement(true);
                dma_stream->enableTransferCompleteInterrupt(true);
            }
            //Start
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED)) return false;
            // send device address with write access
            REG->DR = i2c_addr << 1;
            if (!waitEvent(I2CFlags::ADDRESS_SENT)) return false;
            clearAddress();

            REG->DR = reg_addr;
            if (!waitEvent(I2CFlags::TRANSFER_REGISTER_EMPTY)) return false;

            REG->CR2 |= (1 << 11); // DMAEN (Enable DMA requests)
            //repeated start and read
            REG->CR1 |= 1 << 8;
            if (!waitEvent(I2CFlags::START_BIT_GENERATED))return false;
            REG->DR = (i2c_addr << 1) | 1u; // Device Addr (For read access we set 0th bit to 1)
            if (!waitEvent(I2CFlags::ADDRESS_SENT))return false;

            if (dma_stream) {
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
    using I2C2 = I2C<0x4000'5C00>;
    using I2C3 = I2C<0x4000'5800>;
}
#endif //BIPEDALV1_I2C_H
