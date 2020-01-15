#include "sam.h"
#include "Driver_MCI.h"
#include "RTE_Device.h"

#define ARM_MCI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* driver version */
#define MCI_FLAG_INITIALIZED        (1 << 0)
#define MCI_FLAG_POWERED            (1 << 1)
#define MCI_FLAG_OPEN_DRAIN         (1 << 2)
#define MCI_FLAG_TRANSFER_WRITE     (1 << 3)
#define MCI_FLAG_TRANSFER_STREAM    (1 << 4)
#define MCI_FLAG_RESPONSE           (1 << 5)
#define MCI_FLAG_RESPONSE_LONG      (1 << 6)

#define MCI_PINS (PIO_PA26 | PIO_PA27 | PIO_PA28 | PIO_PA29 | PIO_PA30 | PIO_PA31)
#define MCI_CLKDIV(baud) (SystemCoreClock / (2 * baud) - 1)
#define MCI_ERROR_INTERRUPTS (HSMCI_IDR_RINDE | HSMCI_IDR_RDIRE | HSMCI_IDR_RCRCE | HSMCI_IDR_RENDE | HSMCI_IDR_RTOE | HSMCI_IDR_DCRCE | HSMCI_IDR_DTOE | HSMCI_IDR_CSTOE)

extern uint32_t SystemCoreClock;

//Driver Version
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_MCI_API_VERSION,
    ARM_MCI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MCI_CAPABILITIES DriverCapabilities =
{
#if (RTE_MCI_CD == 1)
    1, /* cd_state          */
#else
    0, /* cd_state          */
#endif
    0, /* cd_event          */
    1, /* vdd               */
    0, /* vdd_1v8           */
    0, /* vccq              */
    0, /* vccq_1v8          */
    0, /* vccq_1v2          */
    1, /* data_width_4      */
    0, /* data_width_8      */
    0, /* data_width_4_ddr  */
    0, /* data_width_8_ddr  */
    0, /* high_speed        */
    0, /* uhs_signaling     */
    0, /* uhs_tuning        */
    0, /* uhs_sdr50         */
    0, /* uhs_sdr104        */
    0, /* uhs_ddr50         */
    0, /* uhs_driver_type_a */
    0, /* uhs_driver_type_c */
    0, /* uhs_driver_type_d */
    0, /* sdio_interrupt    */
    0, /* read_wait         */
    0, /* suspend_resume    */
    0, /* mmc_interrupt     */
    0, /* mmc_boot          */
    0, /* ccs               */
    0  /* ccs_timeout       */
};


static ARM_DRIVER_VERSION MCI_GetVersion(void);
static ARM_MCI_CAPABILITIES MCI_GetCapabilities(void);
static int32_t MCI_Initialize (ARM_MCI_SignalEvent_t cb_event);
static int32_t MCI_Uninitialize(void);
static int32_t MCI_PowerControl(ARM_POWER_STATE state);
static int32_t MCI_CardPower(uint32_t voltage);
static int32_t MCI_ReadCD(void);
static int32_t MCI_ReadWP(void);
static int32_t MCI_SendCommand(uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response);
static int32_t MCI_SetupTransfer(uint8_t  *data, uint32_t block_count, uint32_t block_size, uint32_t mode);
static int32_t MCI_AbortTransfer(void);
static int32_t MCI_Control(uint32_t control, uint32_t arg);
static ARM_MCI_STATUS MCI_GetStatus(void);
static void MCI_Handler(void);
static inline void MCI_RaiseEvent(const uint32_t event);

#if (RTE_MCI == 1)
static ARM_MCI_SignalEvent_t MCI_event = 0;
static uint32_t MCI_flags = 0;
static ARM_MCI_STATUS MCI_status = {0};
static uint32_t *MCI_response;

ARM_DRIVER_MCI Driver_MCI =
{
    MCI_GetVersion,
    MCI_GetCapabilities,
    MCI_Initialize,
    MCI_Uninitialize,
    MCI_PowerControl,
    MCI_CardPower,
    MCI_ReadCD,
    MCI_ReadWP,
    MCI_SendCommand,
    MCI_SetupTransfer,
    MCI_AbortTransfer,
    MCI_Control,
    MCI_GetStatus
};

void HSMCI_Handler(void) { MCI_Handler(); }
#endif

ARM_DRIVER_VERSION MCI_GetVersion(void)
{
    return DriverVersion;
}

ARM_MCI_CAPABILITIES MCI_GetCapabilities(void)
{
    return DriverCapabilities;
}

int32_t MCI_Initialize (ARM_MCI_SignalEvent_t cb_event)
{
    if (MCI_flags & MCI_FLAG_INITIALIZED)
        return ARM_DRIVER_OK;

    //Connect pins to peripheral C
    PIOA->PIO_ABCDSR[0] &= ~MCI_PINS;
    PIOA->PIO_ABCDSR[1] |=  MCI_PINS;

    //Set pin control to peripheral
    PIOA->PIO_PDR = MCI_PINS;

#if (RTE_MCI_CD == 1)
    //Set card detect to input pin, enable pull up resistor
    RTE_MCI_CD_PIO->PIO_PER = RTE_MCI_CD_PIN;
    RTE_MCI_CD_PIO->PIO_ODR = RTE_MCI_CD_PIN;
    RTE_MCI_CD_PIO->PIO_PUER = RTE_MCI_CD_PIN;
#endif

    //Register event
    MCI_event = cb_event;

    //Set initialized flag
    MCI_flags = MCI_FLAG_INITIALIZED;

    return ARM_DRIVER_OK;
}

int32_t MCI_Uninitialize(void)
{
    if (MCI_flags & MCI_FLAG_INITIALIZED)
    {
        //Return pin control to poi controller
        PIOA->PIO_PER = MCI_PINS;

#if (RTE_MCI_CD == 1)
    //Disable card detect input pin pull up resistor
    RTE_MCI_CD_PIO->PIO_PUDR = RTE_MCI_CD_PIN;
#endif
        //Unregister event
        MCI_event = 0;

        //Clear flags
        MCI_flags = 0;
    }


    return ARM_DRIVER_OK;
}


int32_t MCI_PowerControl(ARM_POWER_STATE state)
{
    //Check if driver is initialized
    if ((MCI_flags & MCI_FLAG_INITIALIZED) == 0)
        return ARM_DRIVER_ERROR;

    switch (state)
    {
    case ARM_POWER_OFF:

        //Disable HSMCI
        HSMCI->HSMCI_CR = HSMCI_CR_MCIDIS;

        //Disable IRQ
        NVIC_DisableIRQ(HSMCI_IRQn);

        //Disable RX and TX DMA channels
        HSMCI->HSMCI_MR &= ~HSMCI_MR_PDCMODE;
        HSMCI->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;

        //Disable peripheral clock
        PMC->PMC_PCDR0 = PMC_PCDR0_PID18;

        //Clear powered flag
        MCI_flags &= ~MCI_FLAG_POWERED;

        break;

    case ARM_POWER_FULL:
        //Enable peripheral clock
        PMC->PMC_PCER0 = PMC_PCER0_PID18;

        //Reset HSMCI peripheral
        HSMCI->HSMCI_CR = HSMCI_CR_SWRST;

        //Enable RX and TX DMA channels
        HSMCI->HSMCI_PTCR = HSMCI_PTCR_RXTEN | HSMCI_PTCR_TXTEN;
        HSMCI->HSMCI_MR |= HSMCI_MR_PDCMODE;

        //Enable IRQ
        NVIC_ClearPendingIRQ(HSMCI_IRQn);
        NVIC_EnableIRQ(HSMCI_IRQn);

        //Set data timeout
        HSMCI->HSMCI_DTOR = HSMCI_DTOR_DTOCYC(1) | HSMCI_DTOR_DTOMUL_65536;

        //Set completion signal time out
        HSMCI->HSMCI_CSTOR = HSMCI_CSTOR_CSTOCYC(1) | HSMCI_CSTOR_CSTOMUL_65536;

        //Enable HSMCI
        HSMCI->HSMCI_CR = HSMCI_CR_MCIEN;

        //Set powered flag
        MCI_flags |= MCI_FLAG_POWERED;
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t MCI_CardPower(uint32_t voltage)
{
    switch (voltage & ARM_MCI_POWER_VDD_Msk)
    {
    case ARM_MCI_POWER_VDD_OFF:
        break;

    case ARM_MCI_POWER_VDD_3V3:
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t MCI_ReadCD(void)
{
#if (RTE_MCI_CD == 1)
    if (RTE_MCI_CD_PIO->PIO_PDSR & RTE_MCI_CD_PIN)
        return 0;
    else
        return 1;
#else
    return ARM_DRIVER_ERROR_UNSUPPORTED;
#endif
}

int32_t MCI_ReadWP(void)
{
#if (RTE_MCI_WP == 1)
    if (RTE_MCI_WP_PIO->PIO_PDSR & RTE_MCI_WP_PIN)
        return 0;
    else
        return 1;
#else
    return ARM_DRIVER_ERROR_UNSUPPORTED;
#endif
}

int32_t MCI_SendCommand(uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response)
{
    if ((flags & ARM_MCI_RESPONSE_Msk) && (response == NULL))
        return ARM_DRIVER_ERROR_PARAMETER;

    if (MCI_status.command_active | MCI_status.transfer_active)
        return ARM_DRIVER_ERROR_BUSY;

    MCI_status.command_active = 1;

    //Set command number
    cmd = HSMCI_CMDR_CMDNB(cmd);

    //Set open drain if set
    if (MCI_flags & MCI_FLAG_OPEN_DRAIN)
        cmd |= HSMCI_CMDR_OPDCMD_OPENDRAIN;

    //Special commands
    if (flags & ARM_MCI_CARD_INITIALIZE)
        cmd |= HSMCI_CMDR_SPCMD_INIT;
    else if (flags & ARM_MCI_INTERRUPT_COMMAND)
        cmd |= HSMCI_CMDR_SPCMD_IT_CMD;
    else if (flags & ARM_MCI_INTERRUPT_RESPONSE)
        cmd |= HSMCI_CMDR_SPCMD_IT_RESP;
    else if (flags & ARM_MCI_BOOT_OPERATION)
        cmd |= HSMCI_CMDR_SPCMD_BOR;

    //Start data transfer
    if (flags & ARM_MCI_TRANSFER_DATA)
    {
        //Set transfer direction
        if (MCI_flags & MCI_FLAG_TRANSFER_WRITE)
            cmd |= HSMCI_CMDR_TRDIR_WRITE | HSMCI_CMDR_TRCMD_START_DATA;
        else
            cmd |= HSMCI_CMDR_TRDIR_READ | HSMCI_CMDR_TRCMD_START_DATA;

        if (MCI_flags & MCI_FLAG_TRANSFER_STREAM)
            cmd |= HSMCI_CMDR_TRTYP_STREAM;
        else
            cmd |= HSMCI_CMDR_TRTYP_MULTIPLE;
    }

    //Set response type
    MCI_flags &= ~(MCI_FLAG_RESPONSE | MCI_FLAG_RESPONSE_LONG);

    switch (flags & ARM_MCI_RESPONSE_Msk)
    {
    case ARM_MCI_RESPONSE_NONE:
        cmd |= HSMCI_CMDR_RSPTYP_NORESP;
        break;

    case ARM_MCI_RESPONSE_SHORT:
        cmd |= HSMCI_CMDR_RSPTYP_48_BIT;
        MCI_flags |= MCI_FLAG_RESPONSE;
        break;

    case ARM_MCI_RESPONSE_SHORT_BUSY:
        cmd |= HSMCI_CMDR_RSPTYP_R1B;
        MCI_flags |= MCI_FLAG_RESPONSE;
        break;

    case ARM_MCI_RESPONSE_LONG:
        cmd |= HSMCI_CMDR_RSPTYP_136_BIT;
        MCI_flags |= (MCI_FLAG_RESPONSE | MCI_FLAG_RESPONSE_LONG);
        break;
    }

    //Store command flags and response pointer
    MCI_response = response;

    //Set argument and command
    HSMCI->HSMCI_ARGR = arg;
    HSMCI->HSMCI_CMDR = cmd;

    //Enable command ready interrupt
    HSMCI->HSMCI_IER = MCI_ERROR_INTERRUPTS | HSMCI_IER_CMDRDY;

    //Enable receive or transmit transfer interrupt
    if (cmd & HSMCI_CMDR_TRCMD_START_DATA)
    {
        MCI_status.transfer_active = 1;
        if (cmd & HSMCI_CMDR_TRDIR_READ)
            HSMCI->HSMCI_IER = MCI_ERROR_INTERRUPTS | HSMCI_IER_ENDRX;
        else
            HSMCI->HSMCI_IER = MCI_ERROR_INTERRUPTS | HSMCI_IER_ENDTX;
    }

    return ARM_DRIVER_OK;
}

int32_t MCI_SetupTransfer(uint8_t *data, uint32_t block_count, uint32_t block_size, uint32_t mode)
{
    if ((data == NULL) || (block_count == 0) || (block_size == 0))
        return ARM_DRIVER_ERROR_PARAMETER;

    if (!(MCI_flags & MCI_FLAG_POWERED))
        return ARM_DRIVER_ERROR;

    if (MCI_status.transfer_active)
        return ARM_DRIVER_ERROR_BUSY;

    if (mode & ARM_MCI_TRANSFER_STREAM)
        return ARM_DRIVER_ERROR_UNSUPPORTED;

    if (mode & ARM_MCI_TRANSFER_WRITE)
    {
        //Setup write transfer
        MCI_flags |= MCI_FLAG_TRANSFER_WRITE;
        HSMCI->HSMCI_TPR = (uint32_t) data;
        HSMCI->HSMCI_TCR = block_count * block_size / 4;
    }
    else
    {
        //Setup read transfer
        MCI_flags &= ~MCI_FLAG_TRANSFER_WRITE;
        HSMCI->HSMCI_RPR = (uint32_t) data;
        HSMCI->HSMCI_RCR = block_count * block_size / 4;
    }

    if (mode & ARM_MCI_TRANSFER_STREAM)
        MCI_flags |= MCI_FLAG_TRANSFER_STREAM;
    else
        MCI_flags &= ~MCI_FLAG_TRANSFER_STREAM;

    return ARM_DRIVER_OK;
}

int32_t MCI_AbortTransfer(void)
{
    return ARM_DRIVER_ERROR;
}

int32_t MCI_Control(uint32_t control, uint32_t arg)
{
    switch (control)
    {
    case ARM_MCI_BUS_SPEED:
        //Set bus speed, arg in bits/s
        HSMCI->HSMCI_MR &= ~HSMCI_MR_CLKDIV_Msk;
        HSMCI->HSMCI_MR |= HSMCI_MR_CLKDIV(MCI_CLKDIV(arg));

        break;

    case ARM_MCI_BUS_SPEED_MODE:

        switch (arg)
        {
        case ARM_MCI_BUS_DEFAULT_SPEED:
            HSMCI->HSMCI_CFG &= ~HSMCI_CFG_HSMODE;
            break;

        case ARM_MCI_BUS_HIGH_SPEED:
            HSMCI->HSMCI_CFG |= HSMCI_CFG_HSMODE;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        break;

    case ARM_MCI_BUS_CMD_MODE:
        /* Implement external pull-up control to support MMC cards in open-drain mode */
        /* Default mode is push-pull and is configured in Driver_MCI0.Initialize()    */
        switch (arg)
        {
        case ARM_MCI_BUS_CMD_PUSH_PULL:
            MCI_flags &= ~MCI_FLAG_OPEN_DRAIN;
            break;

        case ARM_MCI_BUS_CMD_OPEN_DRAIN:
            MCI_flags |= MCI_FLAG_OPEN_DRAIN;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }

        break;

    case ARM_MCI_BUS_DATA_WIDTH:
        switch (arg)
        {
        case ARM_MCI_BUS_DATA_WIDTH_1:
            HSMCI->HSMCI_SDCR &= ~HSMCI_SDCR_SDCBUS_Msk;
            HSMCI->HSMCI_SDCR |= HSMCI_SDCR_SDCBUS_1;
            break;

        case ARM_MCI_BUS_DATA_WIDTH_4:
            HSMCI->HSMCI_SDCR &= ~HSMCI_SDCR_SDCBUS_Msk;
            HSMCI->HSMCI_SDCR |= HSMCI_SDCR_SDCBUS_4;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        break;

    //case ARM_MCI_CONTROL_RESET:

    case ARM_MCI_CONTROL_CLOCK_IDLE:
        break;

    case ARM_MCI_DATA_TIMEOUT:
        break;

    case ARM_MCI_MONITOR_SDIO_INTERRUPT:
        break;

    case ARM_MCI_CONTROL_READ_WAIT:
        break;

    case ARM_MCI_DRIVER_STRENGTH:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

ARM_MCI_STATUS MCI_GetStatus(void)
{
    return MCI_status;
}

void MCI_Handler(void)
{
    const uint32_t temp = HSMCI->HSMCI_SR & HSMCI->HSMCI_IMR;

    if (temp & HSMCI_SR_CMDRDY)
    {
        //Read response
        if (MCI_flags & MCI_FLAG_RESPONSE)
        {
            MCI_response[0] = HSMCI->HSMCI_RSPR[0];

            if (MCI_flags & MCI_FLAG_RESPONSE_LONG)
            {
                MCI_response[1] = HSMCI->HSMCI_RSPR[1];
                MCI_response[2] = HSMCI->HSMCI_RSPR[2];
                MCI_response[3] = HSMCI->HSMCI_RSPR[3];
            }
        }

        //Disable interrupt
        HSMCI->HSMCI_IDR = HSMCI_IDR_CMDRDY;

        MCI_status.command_active = 0;

        //Raise event command complete
        MCI_RaiseEvent(ARM_MCI_EVENT_COMMAND_COMPLETE);
    }
    else if (temp & (HSMCI_SR_ENDRX | HSMCI_SR_ENDTX))
    {
        //Disable interrupt
        HSMCI->HSMCI_IDR = HSMCI_IDR_ENDRX | HSMCI_IDR_ENDTX;

        MCI_status.transfer_active = 0;

        //Raise event transfer complete
        MCI_RaiseEvent(ARM_MCI_EVENT_TRANSFER_COMPLETE);
    }
    else if (temp & (HSMCI_SR_RINDE | HSMCI_SR_RDIRE | HSMCI_SR_RCRCE | HSMCI_SR_RENDE))
    {
        //Disable interrupts
        HSMCI->HSMCI_IDR = HSMCI_IDR_RINDE | HSMCI_IDR_RDIRE | HSMCI_IDR_RCRCE | HSMCI_IDR_RENDE;

        MCI_status.command_error = 1;
        MCI_status.command_active = 0;

        //Raise event command error
        MCI_RaiseEvent(ARM_MCI_EVENT_COMMAND_ERROR);
    }
    else if (temp & HSMCI_SR_RTOE)
    {
        //Disable interrupt
        HSMCI->HSMCI_IDR = HSMCI_IDR_RTOE;

        MCI_status.command_timeout = 1;
        MCI_status.command_active = 0;

        //Raise event command time out
        MCI_RaiseEvent(ARM_MCI_EVENT_COMMAND_TIMEOUT);
    }
    else if(temp & HSMCI_SR_DCRCE)
    {
        //Disable interrupt
        HSMCI->HSMCI_IDR = HSMCI_IDR_DCRCE;

        MCI_status.transfer_error = 1;
        MCI_status.transfer_active = 0;

        //Raise event transfer error
        MCI_RaiseEvent(ARM_MCI_EVENT_TRANSFER_ERROR);
    }
    else if (temp & HSMCI_IMR_DTOE)
    {
        //Disable interrupt
        HSMCI->HSMCI_IDR = HSMCI_IDR_DTOE;

        MCI_status.transfer_timeout = 1;
        MCI_status.transfer_active = 0;

        //Raise event transfer time out
        MCI_RaiseEvent(ARM_MCI_EVENT_TRANSFER_TIMEOUT);
    }
}


void MCI_RaiseEvent(const uint32_t event)
{
    if (MCI_event)
        MCI_event(event);
}
