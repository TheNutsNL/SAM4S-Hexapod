#include "Driver_TWI.h"
#include "RTE_Device.h"

#define TWI_FLAG_INITIALIZED   (1 << 0)
#define TWI_FLAG_POWERED       (1 << 1)
#define TWI_FLAG_ENABLED       (1 << 2)

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
    if (_info.status.state == State::UNINITIALIZED)
    {
        _pinSDA.Enable();
        _pinSCL.Enable();

        //Register event
        _info.signalEvent = signalEvent;

        _info.status.state = State::INITIALIZED;
    }

    return DRIVER_OK;
}

Result TWI::Uninitialize() const
{
    if (_info.status.state == State::INITIALIZED)
    {
        if (_info.status.state >= State::IDLE)
            PowerControl(POWER_OFF);

        //Return control to pio controller
        _pinSDA.Disable();
        _pinSCL.Disable();

        //Set state to uninitialized
        _info.status.state = State::UNINITIALIZED;
        _info.status.error = Error::NONE;
    }

    return DRIVER_OK;
}

Result TWI::PowerControl(PowerState state) const
{
    //Check if driver is initialized
    if ((_info.status.state == State::UNINITIALIZED))
        return DRIVER_ERROR;

    switch (state)
    {
    case POWER_OFF:
        if (_info.status.state >= State::IDLE)
        {
            //Disable IRQ
            NVIC_DisableIRQ(_irq);

            //Disable RX and TX DMA channels
            _twi->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;

            //Disable peripheral clock
            PMC->PMC_PCDR0 = (1 << _irq);

            _info.status.state = State::INITIALIZED;
        }
        break;

    case POWER_FULL:
        if (_info.status.state == State::IDLE)
        {
            //Enable peripheral clock
            PMC->PMC_PCER0 = (1 << _irq);

            //Reset device
            _twi->TWI_CR = TWI_CR_SWRST;

            //Enable IRQ
            NVIC_ClearPendingIRQ(_irq);
            NVIC_EnableIRQ(_irq);

            _info.status.state = State::IDLE;
            _info.status.error = Error::NONE;
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
}

Result TWI::MasterWrite(uint32_t address,  InternalAddress internalAddress, void *data, uint16_t dataCount) const
{
    //Check if TWI is powered
    if (_info.status.state < State::IDLE)
        return DRIVER_ERROR;

    //Check if TWI is not busy
    if (_info.status.state > State::IDLE)
        return DRIVER_ERROR_BUSY;
    
    if (dataCount == 0)
        return DRIVER_ERROR_PARAMETER;
    
    //Set status to master transmit
    _info.status.state = State::MASTER_WRITE;
    _info.status.error = Error::NONE;

    //Copy transfer info
    _info.data = (uint8_t*) data;
    _info.dataCnt = dataCount;
    _info.bytesTransfered = 0;

    //Set slave device and address
    _twi->TWI_MMR = TWI_MMR_DADR(address) | TWI_MMR_IADRSZ(internalAddress.byteCount);
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
    if (_info.status.state < State::IDLE)
        return DRIVER_ERROR;

    //Check if I2C is not busy
    if (_info.status.state > State::IDLE)
        return DRIVER_ERROR_BUSY;

    if (dataCount == 0)
        return DRIVER_ERROR_PARAMETER;

    //Set status to master transmit
    _info.status.state = State::MASTER_READ;
    _info.status.error = Error::NONE;

    //Copy transfer info
    _info.data = (uint8_t*) data;
    _info.dataCnt = dataCount;
    _info.bytesTransfered = 0;

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
        _twi->TWI_IER = TWI_IER_ENDRX | TWI_IER_NACK | TWI_IER_ARBLST;

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
        _twi->TWI_IER = TWI_IER_RXRDY | TWI_IER_NACK | TWI_IER_ARBLST;
    }

    return DRIVER_OK;
}

uint32_t TWI::GetDataCount() const
{
    return _info.bytesTransfered;
}

TWI::Status TWI::GetStatus() const
{
    return _info.status;
}

void TWI::Handler() const
{
    const uint32_t status = _twi->TWI_SR;
    if (status & TWI_SR_ENDTX)
    {
        //Disable DMA channel
        _twi->TWI_PTCR = TWI_PTCR_TXTDIS;

        _info.bytesTransfered = _info.dataCnt - 1;

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
        _twi->TWI_THR = _info.data[_info.bytesTransfered];
        _info.bytesTransfered++;

        if (_info.bytesTransfered == _info.dataCnt)
        {
            //Send stop bit
            _twi->TWI_CR = TWI_CR_STOP;

            //Enable transmission completion interrupt
            _twi->TWI_IER = TWI_IER_TXCOMP;
        }
    }
    else if (status & TWI_SR_TXCOMP)
    {
        Event event;

        //Disable interrupts
        _twi->TWI_IDR = TWI_IDR_TXCOMP | TWI_IDR_ARBLST | TWI_IDR_NACK;

        if (status & TWI_SR_ARBLST)
        {
            _info.status.error = Error::ARBITRATION_LOST;
            event = Event::ARBITRATION_LOST;
        }
        else if (status & TWI_SR_NACK)
        {
            _info.status.error = Error::NOT_ACKNOWLEDGE;
            event = Event::NOT_ACKNOWLEDGE;
        }
        else
        {
            event = Event::TRANSFER_DONE;
            _info.bytesTransfered = _info.dataCnt;
        }

        //Raise event
        if (_info.signalEvent)
            _info.signalEvent(event);

        //Update state
        _info.status.state = State::IDLE;

    }
    else if (status & TWI_SR_ENDRX)
    {
        //Disable DMA channel
        _twi->TWI_PTCR = TWI_PTCR_RXTDIS;

        //Set number of bytes read
        _info.bytesTransfered = _info.dataCnt - 2;
    }
    else if (status & TWI_SR_RXRDY)
    {
        if (_info.bytesTransfered == _info.dataCnt - 2)
        {
            //Set stop bit
            _twi->TWI_CR = TWI_CR_STOP;

            //Read penultimate byte
            _info.data[_info.dataCnt - 2] = _twi->TWI_RHR;
            _info.bytesTransfered = _info.dataCnt - 1;
        }
        else
        {
            //Read last byte
            _info.data[_info.dataCnt - 1] = _twi->TWI_RHR;
            _info.bytesTransfered = _info.dataCnt;

            _info.status.state = State::IDLE;

            //Raise transfer complete event
            if (_info.signalEvent)
                _info.signalEvent(Event::TRANSFER_DONE);
        }
    }
}
