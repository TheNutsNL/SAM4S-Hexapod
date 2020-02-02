#ifndef IO_H_INCLUDED
#define IO_H_INCLUDED

#include "sam.h"

namespace System
{
    namespace Driver
    {
        enum Peripheral
        {
            PeripheralA,
            PeripheralB,
            PeripheralC,
            PeripheralD,
        };

        class PeripheralPin
        {
        public:
            constexpr PeripheralPin(const Peripheral peripheral, Pio * const pio, const uint32_t pin) : _peripheral(peripheral), _pio(pio), _pin(pin) {}
            void Enable() const;
            void Disable() const;

        private:
            const Peripheral _peripheral;
            Pio * const _pio;
            const uint32_t _pin;
        };

        class GPIOPin
        {
        private:
            Pio * const _pio;
            const uint32_t _pin;

        public:
            constexpr GPIOPin(Pio * const pio, const uint32_t pin) : _pio(pio), _pin(pin) { _pio->PIO_PER = _pin; }
            void OutputEnable()  { _pio->PIO_OER = _pin;}
            void OutputDisable() { _pio->PIO_ODR = _pin;}
            void SetHigh() { _pio->PIO_SODR = _pin; }
            void SetLow()  { _pio->PIO_CODR = _pin; }
        };
    }
}
#endif /* IO_H_INCLUDED */
