#include "IO.h"

using namespace System::Driver;

void PeripheralPin::Enable() const
{
    switch (_peripheral)
    {
    case PeripheralA:
        _pio->PIO_ABCDSR[0] &= ~_pin;
        _pio->PIO_ABCDSR[1] &= ~_pin;
        break;

    case PeripheralB:
        _pio->PIO_ABCDSR[0] |= ~_pin;
        _pio->PIO_ABCDSR[1] &= ~_pin;
        break;

    case PeripheralC:
        _pio->PIO_ABCDSR[0] &= ~_pin;
        _pio->PIO_ABCDSR[1] |= _pin;
        break;

    case PeripheralD:
        _pio->PIO_ABCDSR[0] |= _pin;
        _pio->PIO_ABCDSR[1] |= _pin;
        break;
    }

    //Set pin control to peripheral
    _pio->PIO_PDR = _pin;

}

void PeripheralPin::Disable() const
{
    //Set pin control to pio controller
    _pio->PIO_PER = _pin;

}
