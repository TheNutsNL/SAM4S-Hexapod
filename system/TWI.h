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

        enum class PowerState
        {
            OFF,
            LOW,
            FULL,

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
            enum class DeviceState
            {
                UNINITIALIZED = 0,
                INITIALIZED = 1,
                IDLE = 2,
            };


            struct TransferStatus
            {
                bool busy : 1;
                bool master : 1;
                bool read : 1;
                bool generalCall : 1;
                bool arbitrationLost : 1;
                bool notAcknowledge : 1;
            };

            struct TransferInfo
            {
                uint8_t *data;
                uint16_t dataCount;
                uint16_t bytesTransfered;
                TransferStatus status;
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

            constexpr TWI(Twi *const twi, const IRQn_Type irq, const PeripheralPin pinSDA, const PeripheralPin pinSCL) : _twi(twi), _irq(irq), _pinSDA(_pinSDA), _pinSCL(pinSCL), _state(DeviceState::UNINITIALIZED), _transfer({0}), _signalEvent(0) {}
            Result Initialize(SignalEvent signalEvent) const;
            Result PowerControl(PowerState state) const;
            Result SetSpeed(uint32_t baud) const ;
            uint32_t GetDataCount() const;
            TransferStatus GetStatus() const;
            Result MasterWrite(uint32_t address, InternalAddress internalAddress, void *data, uint16_t dataCount) const;
            Result MasterRead(uint32_t address, InternalAddress internalAddress, void *data, uint16_t dataCount) const;
            Result Uninitialize() const;

            void Handler() const;
        private:

            Twi *const _twi;
            const IRQn_Type _irq;
            const PeripheralPin _pinSDA;
            const PeripheralPin _pinSCL;

            mutable DeviceState _state;
            mutable TransferInfo _transfer;
            mutable SignalEvent _signalEvent;

            constexpr uint32_t CalculateSpeedSetting(uint32_t baud);
        };
    }
}

#endif /* DRIVER_TWI_H_INCLUDED */
