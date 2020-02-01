#include "TWI.h"
#include "RTE_Device.h"

using namespace System::Driver;

extern uint32_t SystemCoreClock;

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

//Setup TWI0
#if (RTE_TWI0)
TWI Driver_TWI0(TWI0, TWI0_IRQn, PeripheralPin(PeripheralA, PIOA, PIO_PA3A_TWD0), PeripheralPin(PeripheralA, PIOA, PIO_PA4A_TWCK0));

extern "C"
{
    void TWI0_Handler() { Driver_TWI0.Handler(); }
}
#endif

//Setup TWI1
#if (RTE_TWI1)
TWI Driver_TWI1(TWI1, TWI1_IRQn, PeripheralPin(PeripheralA, PIOB, PIO_PB4A_TWD1), PeripheralPin(PeripheralA, PIOB, PIO_PB5A_TWCK1));

extern "C"
{
    void TWI1_Handler(void) { Driver_TWI1 Handler();}
}
#endif


Result TWI::Initialize(SignalEvent signalEvent) const
{
    if (_state == DeviceState::UNINITIALIZED)
    {
        _pinSDA.Enable();
        _pinSCL.Enable();

        //Register event
        _signalEvent = signalEvent;

        _state = DeviceState::INITIALIZED;
    }

    return DRIVER_OK;
}

Result TWI::Uninitialize() const
{
    if (_state == DeviceState::INITIALIZED)
    {
        if (_state >= DeviceState::IDLE)
            PowerControl(PowerState::OFF);

        //Return control to pio controller
        _pinSDA.Disable();
        _pinSCL.Disable();

        //Set state to uninitialized
        _state = DeviceState::UNINITIALIZED;
    }

    return DRIVER_OK;
}

Result TWI::PowerControl(PowerState state) const
{
    //Check if driver is initialized
    if ((_state == DeviceState::UNINITIALIZED))
        return DRIVER_ERROR;

    switch (state)
    {
    case PowerState::OFF:
        if (_state >= DeviceState::IDLE)
        {
            //Disable IRQ
            NVIC_DisableIRQ(_irq);

            //Disable RX and TX DMA channels
            _twi->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;

            //Disable peripheral clock
            PMC->PMC_PCDR0 = (1 << _irq);

            _state = DeviceState::INITIALIZED;
        }
        break;

    case PowerState::FULL:
        if (_state == DeviceState::IDLE)
        {
            //Enable peripheral clock
            PMC->PMC_PCER0 = (1 << _irq);

            //Reset device
            _twi->TWI_CR = TWI_CR_SWRST;

            //Enable IRQ
            NVIC_ClearPendingIRQ(_irq);
            NVIC_EnableIRQ(_irq);

            _state = DeviceState::IDLE;
            _transfer = {0};
        }
        break;

    default:
        return DRIVER_ERROR_UNSUPPORTED;
    }

    return DRIVER_OK;
}

Result TWI::SetSpeed(uint32_t baud) const
{
    const uint32_t value = SystemCoreClock / (2 * baud) - 4;;
    _twi->TWI_CWGR = TWI_CWGR_CHDIV(value) | TWI_CWGR_CLDIV(value);

    return Result::DRIVER_OK;
}

Result TWI::MasterWrite(uint32_t address,  InternalAddress internalAddress, void *data, uint16_t dataCount) const
{
    //Check if TWI is powered
    if (_state != DeviceState::IDLE)
        return DRIVER_ERROR;

    //Check if TWI is not busy
    if (_transfer.status.busy)
        return DRIVER_ERROR_BUSY;

    if (dataCount == 0)
        return DRIVER_ERROR_PARAMETER;

    //Set status to master transmit
    _transfer.status.master = 1;
    _transfer.status.read = 0;
    _transfer.status.generalCall = 0;
    _transfer.status.arbitrationLost = 0;
    _transfer.status.notAcknowledge = 0;

    //Copy transfer info
    _transfer.data = (uint8_t*) data;
    _transfer.dataCount = dataCount;
    _transfer.bytesTransfered = 0;

    //Set slave device and address
    _twi->TWI_MMR = TWI_MMR_DADR(address) | TWI_MMR_IADRSZ(internalAddress.byteCount);

    if (internalAddress.byteCount)
        _twi->TWI_IADR = TWI_IADR_IADR(internalAddress.value);

    //Set master mode
    _twi->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;

    if (dataCount > 1)
    {
        //Setup DMA channel
        _twi->TWI_TPR = (uint32_t) data;
        _twi->TWI_TCR = dataCount - 1;

        //Enable DMA channel and interrupts
        _twi->TWI_PTCR = TWI_PTCR_TXTEN;
        _twi->TWI_IER = TWI_IER_ENDTX | TWI_IER_TXCOMP | TWI_IER_NACK | TWI_IER_ARBLST;
    }
    else
    {
        //Enable transmit interrupts
        _twi->TWI_IER = TWI_IER_TXRDY | TWI_IER_NACK | TWI_IER_ARBLST;
    }


    return DRIVER_OK;
}

Result TWI::MasterRead(uint32_t address, InternalAddress internalAddress, void *data, uint16_t dataCount) const
{
    //Check if I2C is powered
    if (_state != DeviceState::IDLE)
        return DRIVER_ERROR;

    //Check if I2C is not busy
    if (_transfer.status.busy)
        return DRIVER_ERROR_BUSY;

    if (dataCount == 0)
        return DRIVER_ERROR_PARAMETER;

    //Set status to master read
    _transfer.status.busy = 1;
    _transfer.status.master = 1;
    _transfer.status.read = 1;
    _transfer.status.generalCall = 0;
    _transfer.status.arbitrationLost = 0;
    _transfer.status.notAcknowledge = 0;

    //Copy transfer info
    _transfer.data = (uint8_t*) data;
    _transfer.dataCount = dataCount;
    _transfer.bytesTransfered = 0;

    //Set slave device address
    _twi->TWI_MMR = TWI_MMR_DADR(address) | TWI_MMR_MREAD | TWI_MMR_IADRSZ(internalAddress.byteCount);

    if (internalAddress.byteCount)
        _twi->TWI_IADR = TWI_IADR_IADR(internalAddress.value);

    //Set master mode
    _twi->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;

    if (dataCount > 2)
    {
        //Setup DMA channel
        _twi->TWI_RPR = (uint32_t) data;
        _twi->TWI_RCR = dataCount - 2;

        //Enable DMA channel and interrupt
        _twi->TWI_PTCR = TWI_PTCR_RXTEN;
        _twi->TWI_IER = TWI_IER_ENDRX | TWI_IER_TXCOMP | TWI_IER_NACK | TWI_IER_ARBLST;

        //Start transfer
        _twi->TWI_CR = TWI_CR_START;
    }
    else
    {
        if (dataCount == 1)
            //Start and stop read transfer
            _twi->TWI_CR = TWI_CR_START | TWI_CR_STOP;
        else
            //Start read transfer
            _twi->TWI_CR = TWI_CR_START;

        //Enable interrupts
        _twi->TWI_IER = TWI_IER_RXRDY | TWI_IER_TXCOMP | TWI_IER_NACK | TWI_IER_ARBLST;
    }

    return DRIVER_OK;
}

uint32_t TWI::GetDataCount() const
{
    return _transfer.bytesTransfered;
}

TWI::TransferStatus TWI::GetStatus() const
{
    return _transfer.status;
}

void TWI::Handler() const
{
    const uint32_t status = _twi->TWI_SR;
    if (status & TWI_SR_ENDTX)
    {
        //Disable DMA channel
        _twi->TWI_PTCR = TWI_PTCR_TXTDIS;

        _transfer.bytesTransfered = _transfer.dataCount - 1;

        //Disable DMA interrupt
        _twi->TWI_IDR = TWI_IDR_ENDTX;

        //Enable transmit holding register ready interrupt
        _twi->TWI_IER = TWI_IER_TXRDY;

    }
    else if (status & TWI_SR_TXRDY)
    {
        //Disable transmit ready interrupt
        _twi->TWI_IDR = TWI_IDR_TXRDY;

        //Send last byte
        _twi->TWI_THR = _transfer.data[_transfer.bytesTransfered++];

        if (_transfer.bytesTransfered == _transfer.dataCount)
        {
            //Send stop bit
            _twi->TWI_CR = TWI_CR_STOP;

            //Enable transmission completion interrupt
            _twi->TWI_IER = TWI_IER_TXCOMP;
        }
    }
    else if (status & TWI_SR_ENDRX)
    {
        //Disable DMA channel
        _twi->TWI_PTCR = TWI_PTCR_RXTDIS;

        //Set number of bytes read
        _transfer.bytesTransfered = _transfer.dataCount - 2;

        //Disable DMA interrupt
        _twi->TWI_IDR = TWI_IDR_ENDRX;

        //Enable Receive Holding Register Ready
        _twi->TWI_IER = TWI_IER_RXRDY;
    }
    else if (status & TWI_SR_RXRDY)
    {
        _transfer.data[_transfer.bytesTransfered++] = _twi->TWI_RHR;

        //Set stop bit on penultimate byte
        if (_transfer.bytesTransfered == _transfer.dataCount - 1)
        {

            _twi->TWI_CR = TWI_CR_STOP;
        }
        else if (_transfer.bytesTransfered == _transfer.dataCount)
        {
            //Disable Receive Holding Register Ready interrupt
            _twi->TWI_IDR = TWI_IDR_RXRDY;
        }
    }
    else if (status & TWI_SR_TXCOMP)
    {
        Event event;

        //Disable interrupts
        _twi->TWI_IDR = TWI_IDR_TXCOMP | TWI_IDR_ARBLST | TWI_IDR_NACK;

        if (status & TWI_SR_ARBLST)
        {
            _transfer.status.arbitrationLost = 1;
            event = Event::ARBITRATION_LOST;
        }
        else if (status & TWI_SR_NACK)
        {
            _transfer.status.notAcknowledge = 1;
            event = Event::NOT_ACKNOWLEDGE;
        }
        else
        {
            event = Event::TRANSFER_DONE;
        }

        //Remove busy flag
        _transfer.status.busy = 0;

        //Raise event
        if (_signalEvent)
            _signalEvent(event);
    }
}
