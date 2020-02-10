#include "PCA9685.h"

using namespace System::Driver;
using namespace Device;

union MODE1
{
    uint8_t reg;
    struct
    {
        uint8_t ALLCALL : 1;
        uint8_t SUB3 : 1;
        uint8_t SUB2 : 1;
        uint8_t SUB1 : 1;
        uint8_t SLEEP: 1;
        uint8_t AI: 1;
        uint8_t EXTCLK: 1;
        uint8_t RESTART: 1;
    };
};

union MODE2
{
    uint8_t reg;
    struct
    {
        uint8_t OUTNE : 2;
        uint8_t OPUTDRV : 1;
        uint8_t OCH : 1;
        uint8_t INVRT : 1;
    };
};

void PCA9685::Reset() const
{
    MODE1 mode1;

    //Write new state of mode1
    mode1.reg = 0;
    mode1.AI = 1;
    mode1.SLEEP = 1;

    Write(Register::MODE1, &mode1.reg, 1);
}

void PCA9685::Sleep(bool enable) const
{
    MODE1 mode1;

    //Read state of mode1
    Read(Register::MODE1, &mode1.reg, 1);

    //Update sleep mode
    if (enable)
        mode1.SLEEP = 1;
    else
        mode1.SLEEP = 0;

    //Write new state of mode1
    Write(Register::MODE1, &mode1.reg, 1);
}

void PCA9685::SetPrescaler(uint8_t prescaler) const
{
    MODE1 oldMode1, newMode1;

    //Enable sleep mode before changing prescaler
    Read(Register::MODE1, &oldMode1.reg, 1);
    newMode1.reg = oldMode1.reg;
    newMode1.SLEEP = 1;
    Write(Register::MODE1, &newMode1.reg, 1);

    //Set prescaler
    Write(Register::PRE_SCALE, &prescaler, 1);

    //Restore state of MODE1 register
    if (oldMode1.reg != newMode1.reg)
    {
        Write(Register::MODE1, &oldMode1.reg, 1);
    }

}

uint8_t PCA9685::CalculatePrescaler(float frequency) const
{
    //Clip frequency
    if (frequency < _clockFrequency / (4096 * 256))
    {
        frequency = _clockFrequency / (4096 * 256);
    }
    else if (frequency > _clockFrequency / (4096 * 4))
    {
        frequency = _clockFrequency / (4096 * 4);
    }

    return (float) _clockFrequency / (4096 * frequency) - 1;;
}

void PCA9685::SetTimeOn(Channel channel, uint16_t value) const
{
    if (value > 4095)
        value = 4095;

    Write((Register) ((uint32_t) Register::LED0_ON_L + 4 * (uint32_t) channel), &value, 2);
}

void PCA9685::SetTimeOff(Channel channel, uint16_t value) const
{
    if (value > 4095)
        value = 4095;

    Write((Register) ((uint32_t) Register::LED0_OFF_L + 4 * (uint32_t) channel), &value, 2);
}

void PCA9685::Write(PCA9685::Register reg, const void *data, uint16_t dataCount) const
{
     _twi->MasterWrite(_address, {(uint8_t) reg, 1}, data, dataCount);

    while (_twi->GetStatus().busy);
}

void PCA9685::Read(PCA9685::Register reg, void *data, uint16_t dataCount) const
{
    _twi->MasterRead(_address, {(uint8_t) reg, 1}, data, dataCount);

    while (_twi->GetStatus().busy);
}
