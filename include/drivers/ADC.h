#ifndef BIPEDALV1_ADC_H
#define BIPEDALV1_ADC_H
#include "GPIO.h"
#include "MemoryMap.h"

namespace F411::ADC {
    enum class Resolution {
        VeryHigh = 0b00,
        High = 0b01,
        Low = 0b10,
        VeryLow = 0b11,
    };

    enum class SampleTime {
        Cycles3,
        Cycles15,
        Cycles28,
        Cycles56,
        Cycles84,
        Cycles112,
        Cycles144,
        Cycles480,
    };

    template<typename Pin>
    struct ADCTraits;

    template<>
    struct ADCTraits<Pins::A0> {
        static constexpr uint8_t channel = 0;
    };

    template<>
    struct ADCTraits<Pins::A1> {
        static constexpr uint8_t channel = 1;
    };

    template<>
    struct ADCTraits<Pins::A4> {
        static constexpr uint8_t channel = 4;
    };

    template<>
    struct ADCTraits<Pins::A5> {
        static constexpr uint8_t channel = 5;
    };

    template<>
    struct ADCTraits<Pins::B0> {
        static constexpr uint8_t channel = 8;
    };

    template<>
    struct ADCTraits<Pins::B1> {
        static constexpr uint8_t channel = 9;
    };

    template<typename... SensorPins>
    class ADC {
    public:
        uint16_t buffer[sizeof...(SensorPins)] = {};

        void enable(Resolution res, SampleTime sample_time = SampleTime::Cycles480) {
            MemoryMap::RCC1->enablePeripheral(MemoryMap::APB2Peripheral::ADC1);
            (SensorPins::enableAnalogMode(), ...);
            auto adc = MemoryMap::ADC;

            adc->CR1 &= ~(0b11 << 24);
            adc->CR1 |= (static_cast<unsigned int>(res) << 24);
            adc->CR1 |= 0b1 << 8;
            adc->CR2 |= (0b1 << 10);

            constexpr uint8_t length = sizeof...(SensorPins);
            const uint8_t channels[length] = {ADCTraits<SensorPins>::channel...};

            setSequence(channels, length);
            setSampleTime(sample_time, channels, length);
            adc->CR2 |= (0b1 << 0);
        }

        uint16_t *read() {
            start();
            constexpr uint8_t length = sizeof...(SensorPins);
            for (uint8_t i = 0; i < length; i++) {
                while (!isEndOfChannel()) {
                }
                buffer[i] = MemoryMap::ADC->DR;
            }
            return buffer;
        }

        void enableDMARead() {
            auto adc = MemoryMap::ADC;
            MemoryMap::RCC1->enablePeripheral(MemoryMap::AHB1Peripheral::DMA2);

            MemoryMap::DMAStream *dma_stream = MemoryMap::DMA2->STREAMS[0].isEnabled()
                                                   ? &MemoryMap::DMA2->STREAMS[4]
                                                   : &MemoryMap::DMA2->STREAMS[0];
            dma_stream->setDataTransferMode(MemoryMap::DMAStream::TransferDirection::PERIPHERAL_TO_MEMORY);
            dma_stream->setPeripheralAddress(&adc->DR);
            dma_stream->setMemoryAddress(&buffer[0]);
            dma_stream->setDataLength(sizeof...(SensorPins));
            if (sizeof...(SensorPins) <= 4)
                dma_stream->enableFIFO(true, static_cast<MemoryMap::DMAStream::FIFOStorageWord>(sizeof...(SensorPins)));
            dma_stream->setSize(MemoryMap::DMAStream::DMAMemorySize::HALF_WORD,
                                MemoryMap::DMAStream::DMAMemorySize::HALF_WORD);
            dma_stream->enableMemoryIncrementMode(true);
            dma_stream->enableCircularMode(true);
            dma_stream->setChannel(MemoryMap::DMAStream::Channel::CH0);
            dma_stream->setEnabled(true);

            adc->CR2 |= 0b1 << 1;
            adc->CR2 |= 0b1 << 8 | 0b1 << 9;
            start();
        }

    private:
        void setSequence(const uint8_t *channels, uint8_t length) {
            const auto adc = MemoryMap::ADC;

            adc->SQR1 &= ~(0b1111 << 20);
            adc->SQR1 |= ((length - 1) << 20);

            adc->SQR1 &= (0b1111 << 20);
            adc->SQR2 = 0;
            adc->SQR3 = 0;

            for (uint8_t i = 0; i < length; i++) {
                uint8_t channel = channels[i];

                if (i < 6) {
                    adc->SQR3 |= (channel << (i * 5));
                } else if (i < 12) {
                    adc->SQR2 |= (channel << ((i - 6) * 5));
                } else {
                    adc->SQR1 |= (channel << ((i - 12) * 5));
                }
            }
        }

        void setSampleTime(SampleTime sample_time, const uint8_t *channels, uint8_t length) {
            for (uint8_t i = 0; i < length; i++) {
                if (channels[i] <= 9) {
                    MemoryMap::ADC->SMPR2 &= ~(0b111 << (channels[i] * 3));
                    MemoryMap::ADC->SMPR2 |= static_cast<unsigned int>(sample_time) << (channels[i] * 3);
                } else {
                    MemoryMap::ADC->SMPR1 &= ~(0b111 << ((channels[i] - 10) * 3));
                    MemoryMap::ADC->SMPR1 |= static_cast<unsigned int>(sample_time) << ((channels[i] - 10) * 3);
                }
            }
        }

        static void start() {
            MemoryMap::ADC->CR2 |= 0b1 << 30;
        }

        static bool isEndOfChannel() {
            return (MemoryMap::ADC->SR & 0b10) == 0b10;
        }
    };
}
#endif //BIPEDALV1_ADC_H
