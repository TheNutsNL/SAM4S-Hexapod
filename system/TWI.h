#ifndef DRIVER_TWI_H_INCLUDED
#define DRIVER_TWI_H_INCLUDED

#include "sam.h"
#include "IO.h"

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

        enum class PowerState
        {
            OFF,
            LOW,
            FULL,

        };

        class TWI
        {
        public:
            union TransferStatus
            {
                uint32_t reg;
                struct
                {
                    bool busy : 1;
                    bool master : 1;
                    bool read : 1;
                    bool generalCall : 1;
                    bool arbitrationLost : 1;
                    bool notAcknowledge : 1;
                };
            };

            struct TransferInfo
            {
                uint8_t *data;
                uint16_t dataCount;
                uint16_t bytesTransfered;
            };

            enum class Event
            {
                TRANSFER_DONE,
                //SLAVE_TRANSMIT,
                //SLAVE_RECEIVE,
                NOT_ACKNOWLEDGE,
                //GENERAL_CALL,
                ARBITRATION_LOST,
                //BUS_ERROR,
            };

            struct InternalAddress
            {
                uint32_t value : 24;
                uint8_t byteCount : 2;
            };

            typedef void (*SignalEvent) (Event event);

            constexpr TWI(Twi *const twi, const IRQn_Type irq, const PeripheralPin pinSDA, const PeripheralPin pinSCL) : _twi(twi), _irq(irq), _pinSDA(pinSDA), _pinSCL(pinSCL), _state(DeviceState::UNINITIALIZED), _transferInfo({0}), _transferStatus({0}), _signalEvent(0) {}
            Result Initialize(SignalEvent signalEvent) const;
            Result PowerControl(PowerState state) const;
            Result SetSpeed(uint32_t baud) const ;
            uint32_t GetDataCount() const;
            const TransferStatus GetStatus() const;
            Result MasterWrite(uint32_t address, InternalAddress internalAddress, const void *data, uint16_t dataCount) const;
            Result MasterRead(uint32_t address, InternalAddress internalAddress, void *data, uint16_t dataCount) const;
            Result Uninitialize() const;

            void Handler() const;
        private:
            enum class DeviceState
            {
                UNINITIALIZED = 0,
                INITIALIZED = 1,
                POWERED = 2,
            };

            Twi *const _twi;
            const IRQn_Type _irq;
            const PeripheralPin _pinSDA;
            const PeripheralPin _pinSCL;

            mutable DeviceState _state;
            mutable volatile TransferInfo _transferInfo;
            mutable volatile TransferStatus _transferStatus;
            mutable SignalEvent _signalEvent;
        };
    }
}

#endif /* DRIVER_TWI_H_INCLUDED */
