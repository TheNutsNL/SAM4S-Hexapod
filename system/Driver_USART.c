#include "sam.h"
#include "Driver_USART.h"
#include "RTE_Device.h"

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0)

#define UART_FLAG_INITIALIZED   (1 << 0)
#define UART_FLAG_POWERED       (1 << 1)
#define UART_FLAG_ENABLED       (1 << 2)
#define UART_FLAG_RX_ENABLED    (1 << 3)
#define UART_FLAG_TX_ENABLED    (1 << 4)

extern uint32_t SystemCoreClock;

typedef const struct
{
  Pio* pio;
  uint32_t pinRX;
  uint32_t pinTX;
} UART_Pins;

typedef struct
{
    ARM_USART_SignalEvent_t event;
    uint32_t flags;
    ARM_USART_STATUS status;
    uint32_t rx_num;
    uint32_t tx_num;
} UART_Info;

typedef const struct
{
    Uart *uart;
    uint32_t irqNum;
    UART_Pins pins;
    UART_Info *info;
} UART_Resource;

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_USART_CAPABILITIES UART_DriverCapabilities =
{
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0  /* Signal RI change event: \ref ARM_USART_EVENT_RI */
};


static ARM_DRIVER_VERSION UART_GetVersion(void);
static ARM_USART_CAPABILITIES UART_GetCapabilities(void);
static int32_t UARTx_Initialize(UART_Resource *res, ARM_USART_SignalEvent_t cb_event);
static int32_t UARTx_Uninitialize(UART_Resource *res);
static int32_t UARTx_PowerControl(UART_Resource *res, ARM_POWER_STATE state);
static int32_t UARTx_Send(UART_Resource *res, const void *data, uint32_t num);
static int32_t UARTx_Receive(UART_Resource *res, void *data, uint32_t num);
static uint32_t UARTx_GetTxCount(UART_Resource *res);
static uint32_t UARTx_GetRxCount(UART_Resource *res);
static int32_t UARTx_Control(UART_Resource *res, uint32_t control, uint32_t arg);
static ARM_USART_STATUS UARTx_GetStatus(UART_Resource *res);
static void UARTx_Handler(UART_Resource *res);

//Setup UART0
#if (RTE_UART0)
static UART_Info UART0_info = {0};
static UART_Resource UART0_res =
{
    UART0,
    UART0_IRQn,
    {
        PIOA,
        PIO_PA9,
        PIO_PA10,
    },
    &UART0_info,
};

static int32_t UART0_Initialize(ARM_USART_SignalEvent_t cb_event) { return UARTx_Initialize(&UART0_res, cb_event); }
static int32_t UART0_Uninitialize(void) { return UARTx_Uninitialize(&UART0_res); };
static int32_t UART0_PowerControl(ARM_POWER_STATE state) { return UARTx_PowerControl(&UART0_res, state); }
static int32_t UART0_Send(const void *data, uint32_t num) { return UARTx_Send(&UART0_res, data, num); }
static int32_t UART0_Receive(void *data, uint32_t num) { return UARTx_Receive(&UART0_res, data, num); }
static int32_t UART0_Transfer(const void *data_out, void *data_in, uint32_t num) { return ARM_DRIVER_ERROR_UNSUPPORTED; }
static uint32_t UART0_GetTxCount(void) { return UARTx_GetTxCount(&UART0_res); }
static uint32_t UART0_GetRxCount(void) { return UARTx_GetRxCount(&UART0_res); }
static int32_t UART0_Control(uint32_t control, uint32_t arg) { return UARTx_Control(&UART0_res, control, arg); }
static ARM_USART_STATUS UART0_GetStatus(void) { return UARTx_GetStatus(&UART0_res); }
static int32_t UART0_SetModemControl(ARM_USART_MODEM_CONTROL control) {return ARM_DRIVER_ERROR_UNSUPPORTED; }
static ARM_USART_MODEM_STATUS UART0_GetModemStatus(void) {ARM_USART_MODEM_STATUS sts = {0}; return sts;}
void UART0_Handler(void) { UARTx_Handler(&UART0_res); }

ARM_DRIVER_USART Driver_USART0 =
{
    UART_GetVersion,
    UART_GetCapabilities,
    UART0_Initialize,
    UART0_Uninitialize,
    UART0_PowerControl,
    UART0_Send,
    UART0_Receive,
    UART0_Transfer,
    UART0_GetTxCount,
    UART0_GetRxCount,
    UART0_Control,
    UART0_GetStatus,
    UART0_SetModemControl,
    UART0_GetModemStatus
};
#endif

//Setup UART1
#if (RTE_UART1)
static UART_Info UART1_info = {0};
static UART_Resource UART1_res =
{
    UART1,
    UART1_IRQn,
    {
        PIOB,
        PIO_PB2,
        PIO_PB3,
    },
    &UART1_info,
};

static int32_t UART1_Initialize(ARM_USART_SignalEvent_t cb_event) { return UARTx_Initialize(&UART1_res, cb_event); }
static int32_t UART1_Uninitialize(void) { return UARTx_Uninitialize(&UART1_res); };
static int32_t UART1_PowerControl(ARM_POWER_STATE state) { return UARTx_PowerControl(&UART1_res, state); }
static int32_t UART1_Send(const void *data, uint32_t num) { return UARTx_Send(&UART1_res, data, num); }
static int32_t UART1_Receive(void *data, uint32_t num) { return UARTx_Receive(&UART1_res, data, num); }
static int32_t UART1_Transfer(const void *data_out, void *data_in, uint32_t num) { return ARM_DRIVER_ERROR_UNSUPPORTED; }
static uint32_t UART1_GetTxCount(void) { return UARTx_GetTxCount(&UART1_res); }
static uint32_t UART1_GetRxCount(void) { return UARTx_GetRxCount(&UART1_res); }
static int32_t UART1_Control(uint32_t control, uint32_t arg) { return UARTx_Control(&UART1_res, control, arg); }
static ARM_USART_STATUS UART1_GetStatus(void) { return UARTx_GetStatus(&UART1_res); }
static int32_t UART1_SetModemControl(ARM_USART_MODEM_CONTROL control) {return ARM_DRIVER_ERROR_UNSUPPORTED; }
static ARM_USART_MODEM_STATUS UART1_GetModemStatus(void) {ARM_USART_MODEM_STATUS sts = {0}; return sts;}
void UART1_Handler(void) { UARTx_Handler(&UART1_res); }

ARM_DRIVER_USART Driver_USART1 =
{
    UART_GetVersion,
    UART_GetCapabilities,
    UART1_Initialize,
    UART1_Uninitialize,
    UART1_PowerControl,
    UART1_Send,
    UART1_Receive,
    UART1_Transfer,
    UART1_GetTxCount,
    UART1_GetRxCount,
    UART1_Control,
    UART1_GetStatus,
    UART1_SetModemControl,
    UART1_GetModemStatus
};
#endif

static ARM_DRIVER_VERSION UART_GetVersion(void)
{
    return DriverVersion;
}

static ARM_USART_CAPABILITIES UART_GetCapabilities(void)
{
    return UART_DriverCapabilities;
}

static int32_t UARTx_Initialize(UART_Resource *res, ARM_USART_SignalEvent_t cb_event)
{
    if ((res->info->flags & UART_FLAG_INITIALIZED) == 0)
    {
        Pio *pio = res->pins.pio;
        const uint32_t pins = (res->pins.pinRX | res->pins.pinTX);

        //Connect pins to peripheral A
        pio->PIO_ABCDSR[0] &= ~pins;
        pio->PIO_ABCDSR[1] &= ~pins;

        //Set pin control to peripheral
        pio->PIO_PDR = pins;

        //Register event
        res->info->event = cb_event;

        res->info->flags = UART_FLAG_INITIALIZED;
    }

    return ARM_DRIVER_OK;
}

static int32_t UARTx_Uninitialize(UART_Resource *res)
{
    if (res->info->flags & UART_FLAG_INITIALIZED)
    {
        if (res->info->flags & UART_FLAG_POWERED)
            UARTx_PowerControl(res, ARM_POWER_OFF);

        //Return control to pio controller
        res->pins.pio->PIO_PER = (res->pins.pinRX | res->pins.pinTX);

        //Reset flags
        res->info->flags = 0;
    }

    return ARM_DRIVER_OK;
}

static int32_t UARTx_PowerControl(UART_Resource *res, ARM_POWER_STATE state)
{
    //Check if driver is initialized
    if ((res->info->flags & UART_FLAG_INITIALIZED) == 0)
        return ARM_DRIVER_ERROR;

    switch (state)
    {
    case ARM_POWER_OFF:
        if (res->info->flags & UART_FLAG_POWERED)
        {
            //Disable IRQ
            NVIC_DisableIRQ(res->irqNum);

            //Disable RX and TX DMA channels
            res->uart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

            //Disable peripheral clock
            PMC->PMC_PCDR0 = (1 << res->irqNum);

            res->info->flags &= ~(UART_FLAG_POWERED);
        }

        break;

    case ARM_POWER_FULL:
        if ((res->info->flags & UART_FLAG_POWERED) == 0)
        {
            //Enable peripheral clock
            PMC->PMC_PCER0 = (1 << res->irqNum);

            //Reset receiver, transmitter and status
            res->uart->UART_CR = (UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA);

            //Enable IRQ
            NVIC_ClearPendingIRQ(res->irqNum);
            NVIC_EnableIRQ(res->irqNum);

            res->info->flags |= UART_FLAG_POWERED;
        }


        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t UARTx_Send(UART_Resource *res, const void *data, uint32_t num)
{
    if ((res->info->flags & UART_FLAG_TX_ENABLED) == 0)
        return ARM_DRIVER_ERROR;

    if (res->info->status.tx_busy)
        return ARM_DRIVER_ERROR_BUSY;

    if (num > 0xFFFFFFFF)
        return ARM_DRIVER_ERROR_PARAMETER;

    res->info->status.tx_busy = 1;
    res->info->tx_num = num;

    //Setup DMA
    res->uart->UART_TPR = (uint32_t) data;
    res->uart->UART_TCR = num;

    //Enable interrupts
    res->uart->UART_IER = UART_IER_ENDTX;

    return ARM_DRIVER_OK;
}

int32_t UARTx_Receive(UART_Resource *res, void *data, uint32_t num)
{
    if ((res->info->flags & UART_FLAG_RX_ENABLED) == 0)
        return ARM_DRIVER_ERROR;

    if (res->info->status.rx_busy)
        return ARM_DRIVER_ERROR_BUSY;

    if (num > 0xFFFFFFFF)
        return ARM_DRIVER_ERROR_PARAMETER;

    res->info->status.rx_busy = 1;
    res->info->rx_num = num;

    //Setup DMA
    res->uart->UART_RPR = (uint32_t) data;
    res->uart->UART_RCR = num;

    //Enable interrupts
    res->uart->UART_IER = UART_IER_ENDRX | UART_IER_OVRE | UART_IER_PARE | UART_IER_FRAME;

    return ARM_DRIVER_OK;
}

uint32_t UARTx_GetTxCount(UART_Resource *res)
{
    return res->info->tx_num - res->uart->UART_TCR;
}

uint32_t UARTx_GetRxCount(UART_Resource *res)
{
    return res->info->rx_num - res->uart->UART_RCR;
}

int32_t UARTx_Control(UART_Resource *res, uint32_t control, uint32_t arg)
{
    //Check if driver is powered
    if ((res->info->flags & UART_FLAG_POWERED) == 0)
        return ARM_DRIVER_ERROR;

    switch (control & ARM_USART_CONTROL_Msk)
    {
    case ARM_USART_MODE_ASYNCHRONOUS:
        //Only 8-bit data allowed
        if ((control & ARM_USART_DATA_BITS_Msk) != ARM_USART_DATA_BITS_8)
            return ARM_USART_ERROR_DATA_BITS;

        //Only 1 stop bit allowed
        if ((control & ARM_USART_STOP_BITS_Msk) != ARM_USART_STOP_BITS_1)
            return ARM_USART_ERROR_STOP_BITS;

        //No flow control supported
        if ((control & ARM_USART_FLOW_CONTROL_Msk) != ARM_USART_FLOW_CONTROL_NONE)
            return ARM_USART_ERROR_FLOW_CONTROL;

        //Set parity
        switch (control & ARM_USART_PARITY_Msk)
        {
        case ARM_USART_PARITY_NONE:
            res->uart->UART_MR = UART_MR_PAR_NO;
            break;

        case ARM_USART_PARITY_EVEN:
            res->uart->UART_MR = UART_MR_PAR_EVEN;
            break;

        case ARM_USART_PARITY_ODD:
            res->uart->UART_MR = UART_MR_PAR_ODD;
            break;

        default:
            return ARM_USART_ERROR_PARITY;
        }

        //Set baud rate
        if ((arg > SystemCoreClock / 16)  || (arg == 0))
            return ARM_USART_ERROR_BAUDRATE;

        res->uart->UART_BRGR = SystemCoreClock / (16 * arg);
        break;

    case ARM_USART_ABORT_RECEIVE:
        res->uart->UART_RCR = 0;
        res->info->status.rx_busy = 0;
        res->info->rx_num = 0;
        break;

    case ARM_USART_ABORT_SEND:
        res->uart->UART_TCR = 0;
        res->info->status.tx_busy = 0;
        res->info->tx_num = 0;
        break;

    case ARM_USART_CONTROL_RX:
        if (arg)
        {
            res->uart->UART_CR |=  UART_CR_RXEN;
            res->info->flags |= UART_FLAG_RX_ENABLED;

            //Enable RX DMA channel
            res->uart->UART_PTCR = UART_PTCR_RXTEN;
        }

        else
        {
            res->uart->UART_CR |=  UART_CR_RXDIS;
            res->info->flags &= ~UART_FLAG_RX_ENABLED;

            //Disable RX DMA channel
            res->uart->UART_PTCR = UART_PTCR_RXTDIS;
        }


        break;

    case ARM_USART_CONTROL_TX:
        if (arg)
        {
            res->uart->UART_CR |=  UART_CR_TXEN;
            res->info->flags |= UART_FLAG_TX_ENABLED;

            //Enable TX DMA channel
            res->uart->UART_PTCR = UART_PTCR_TXTEN;
        }
        else
        {
            res->uart->UART_CR |=  UART_CR_TXDIS;
            res->info->flags &= ~UART_FLAG_TX_ENABLED;

            //Disable TX DMA channel
            res->uart->UART_PTCR = UART_PTCR_TXTDIS;
        }


        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

ARM_USART_STATUS UARTx_GetStatus(UART_Resource *res)
{
    return res->info->status;
}

void UARTx_Handler(UART_Resource *res)
{
    const uint32_t tmp = res->uart->UART_IMR & res->uart->UART_SR;
    if (tmp & UART_SR_ENDRX)
    {
        res->info->status.rx_busy = 0;

        //Raise receive complete event
        if (res->info->event)
            res->info->event(ARM_USART_EVENT_RECEIVE_COMPLETE);
    }
    else if (tmp & UART_SR_ENDTX)
    {
        res->info->status.tx_busy = 0;
        res->uart->UART_IDR = UART_IDR_ENDTX;
        //Raise send complete event
        if (res->info->event)
            res->info->event(ARM_USART_EVENT_SEND_COMPLETE);
    }
    else if (tmp & UART_SR_FRAME)
    {
        res->info->status.rx_framing_error = 1;

        //Raise framing error event
        if (res->info->event)
            res->info->event(ARM_USART_EVENT_RX_FRAMING_ERROR);
    }
    else if (res->uart->UART_SR & UART_SR_OVRE)
    {
        res->info->status.rx_overflow = 1;

        //Raise overflow error event
        if (res->info->event)
            res->info->event(ARM_USART_EVENT_RX_OVERFLOW);
    }
    else if (tmp & UART_SR_PARE)
    {
        res->info->status.rx_parity_error = 1;

        //Raise parity error event
        if (res->info->event)
            res->info->event(ARM_USART_EVENT_RX_PARITY_ERROR);
    }
}
