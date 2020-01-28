#ifndef DRIVER_TWI_H_INCLUDED
#define DRIVER_TWI_H_INCLUDED

#include "sam.h"

namespace System
{
    namespace Driver
    {
        enum Result
        {
            DRIVER_OK,
            DRIVER_ERROR,
            DRIVER_ERROR_BUSY,
            DRIVER_ERROR_UNSUPPORTED,
            DRIVER_ERROR_TIMEOUT,
            DRIVER_ERROR_PARAMETER,
        };

        enum PowerState
        {
            POWER_OFF,
            POWER_LOW,
            POWER_FULL,

        };

        enum Peripheral
        {
            PeripheralA,
            PeripheralB,
            PeripheralC,
            PeripheralD,
        };

        class PeripheralPin
        {
        private:
            const Peripheral _peripheral;
            Pio * const _pio;
            const uint32_t _pin;
        public:
            constexpr PeripheralPin(const Peripheral peripheral, Pio * const pio, const uint32_t pin) : _peripheral(peripheral), _pio(pio), _pin(pin) {}
            void Enable() const;
            void Disable() const ;
        };



        class TWI
        {
        public:
            enum class State
            {
                UNINITIALIZED = 0,
                INITIALIZED = 1,
                IDLE = 2,
                MASTER_WRITE = 3,
                MASTER_READ = 4,
            };


            struct Status
            {
                State state : 4;
                bool generalCall : 1;
                bool arbitrationLost : 1;
                bool busError : 1;
            };

            enum class Event
            {
                TRANSFER_DONE,
                TRANSFER_INCOMPLETE,
                SLAVE_TRANSMIT,
                SLAVE_RECEIVE,
                ADDRESS_NACK,
                GENERAL_CALL,
                ARBITRATION_LOST,
                BUS_ERROR,
            };

            struct InternalAddress
            {
                uint32_t value : 24;
                uint8_t byteCount : 2;
            };

            typedef void (*SignalEvent) (Event event);

            constexpr TWI(Twi *const twi, const IRQn_Type irq, const PeripheralPin pinSDA, const PeripheralPin pinSCL) : _twi(twi), _irq(irq), _pinSDA(_pinSDA), _pinSCL(pinSCL), _info() {}
            Result Initialize(SignalEvent signalEvent) const;
            Result PowerControl(PowerState state) const;
            Result SetSpeed(uint32_t baud) const ;
            uint32_t GetDataCount() const;
            Status GetStatus() const;
            Result MasterWrite(uint32_t address, InternalAddress internalAddress, void *data, uint16_t dataCount) const;
            Result MasterRead(uint32_t address, InternalAddress internalAddress, void *data, uint16_t dataCount) const;
            Result Uninitialize() const;

            void Handler() const;
        private:
            typedef struct
            {
                SignalEvent signalEvent;
                Status status;
                uint8_t *data;
                uint32_t dataCnt;
                uint32_t bytesTransfered;
                bool isPending;
            } TWI_Info;

            Twi *const _twi;
            const IRQn_Type _irq;
            const PeripheralPin _pinSDA;
            const PeripheralPin _pinSCL;

            mutable TWI_Info _info;

            constexpr uint32_t CalculateSpeedSetting(uint32_t baud);
        };
    }
}

#endif /* DRIVER_TWI_H_INCLUDED */
