#include "sam.h"
#include "Driver_I2C.h"
#include "RTE_Device.h"

#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* driver version */

#define I2C_FLAG_INITIALIZED   (1 << 0)
#define I2C_FLAG_POWERED       (1 << 1)
#define I2C_FLAG_ENABLED       (1 << 2)

//Driver Version
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
};

//Driver Capabilities
static const ARM_I2C_CAPABILITIES DriverCapabilities =
{
    0  /* supports 10-bit addressing */
};

typedef const struct
{
  Pio* pio;
  uint32_t pinSDA;
  uint32_t pinSCL;
} I2C_Pins;

typedef struct
{
    ARM_I2C_SignalEvent_t event;
    uint32_t flags;
    ARM_I2C_STATUS status;
} I2C_Info;

typedef const struct
{
    Twi *twi;
    uint32_t irqNum;
    I2C_Pins pins;
    I2C_Info *info;
} I2C_Resource;

extern uint32_t SystemCoreClock;

static ARM_DRIVER_VERSION ARM_I2C_GetVersion(void);
static ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities(void);
static int32_t ARM_I2Cx_Initialize(I2C_Resource *res, ARM_I2C_SignalEvent_t cb_event);
static int32_t ARM_I2Cx_Uninitialize(I2C_Resource *res);
static int32_t ARM_I2Cx_PowerControl(I2C_Resource *res, ARM_POWER_STATE state);
static int32_t ARM_I2Cx_MasterTransmit(I2C_Resource *res, uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t ARM_I2Cx_MasterReceive(I2C_Resource *res, uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t ARM_I2Cx_SlaveTransmit(I2C_Resource *res, const uint8_t *data, uint32_t num);
static int32_t ARM_I2Cx_SlaveReceive(I2C_Resource *res, uint8_t *data, uint32_t num);
static int32_t ARM_I2Cx_GetDataCount(I2C_Resource *res);
static int32_t ARM_I2Cx_Control(I2C_Resource *res, uint32_t control, uint32_t arg);
static ARM_I2C_STATUS ARM_I2Cx_GetStatus(I2C_Resource *res);
static void ARM_I2Cx_Handler(I2C_Resource *res);

//Setup I2C0
#if (RTE_I2C0)
static I2C_Info I2C0_info = {0};
static I2C_Resource I2C0_res =
{
    TWI0,
    TWI0_IRQn,
    {
        PIOA,
        PIO_PA3A_TWD0,
        PIO_PA4A_TWCK0,
    },
    &I2C0_info,
};

static int32_t ARM_I2C0_Initialize(ARM_I2C_SignalEvent_t cb_event) { return ARM_I2Cx_Initialize(&I2C0_res, cb_event);}
static int32_t ARM_I2C0_Uninitialize(void) { return ARM_I2Cx_Uninitialize(&I2C0_res);}
static int32_t ARM_I2C0_PowerControl(ARM_POWER_STATE state) { return ARM_I2Cx_PowerControl(&I2C0_res, state);}
static int32_t ARM_I2C0_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) { return ARM_I2Cx_MasterTransmit(&I2C0_res, addr, data, num, xfer_pending);}
static int32_t ARM_I2C0_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) { return ARM_I2Cx_MasterReceive(&I2C0_res, addr, data, num, xfer_pending);}
static int32_t ARM_I2C0_SlaveTransmit(const uint8_t *data, uint32_t num) { return ARM_I2Cx_SlaveTransmit(&I2C0_res, data, num);}
static int32_t ARM_I2C0_SlaveReceive(uint8_t *data, uint32_t num) { return ARM_I2Cx_SlaveReceive(&I2C0_res, data, num);}
static int32_t ARM_I2C0_GetDataCount(void) { return ARM_I2Cx_GetDataCount(&I2C0_res);}
static int32_t ARM_I2C0_Control(uint32_t control, uint32_t arg) { return ARM_I2Cx_Control(&I2C0_res, control, arg);}
static ARM_I2C_STATUS ARM_I2C0_GetStatus(void) { return ARM_I2Cx_GetStatus(&I2C0_res);}
void TWI1_Handler(void) { ARM_I2Cx_Handler(&I2C0_res);}

ARM_DRIVER_I2C Driver_I2C0 =
{
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    ARM_I2C0_Initialize,
    ARM_I2C0_Uninitialize,
    ARM_I2C0_PowerControl,
    ARM_I2C0_MasterTransmit,
    ARM_I2C0_MasterReceive,
    ARM_I2C0_SlaveTransmit,
    ARM_I2C0_SlaveReceive,
    ARM_I2C0_GetDataCount,
    ARM_I2C0_Control,
    ARM_I2C0_GetStatus
};
#endif

//Setup I2C1
#if (RTE_I2C1)
static I2C_Info I2C1_info = {0};
static I2C_Resource I2C1_res =
{
    TWI1,
    TWI1_IRQn,
    {
        PIOB,
        PIO_PB4A_TWD1,
        PIO_PB5A_TWCK1,
    },
    &I2C1_info,
};

static int32_t ARM_I2C1_Initialize(ARM_I2C_SignalEvent_t cb_event) { return ARM_I2Cx_Initialize(&I2C1_res, cb_event);}
static int32_t ARM_I2C1_Uninitialize(void) { return ARM_I2Cx_Uninitialize(&I2C1_res);}
static int32_t ARM_I2C1_PowerControl(ARM_POWER_STATE state) { return ARM_I2Cx_PowerControl(&I2C1_res, state);}
static int32_t ARM_I2C1_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) { return ARM_I2Cx_MasterTransmit(&I2C1_res, addr, data, num, xfer_pending);}
static int32_t ARM_I2C1_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) { return ARM_I2Cx_MasterReceive(&I2C1_res, addr, data, num, xfer_pending);}
static int32_t ARM_I2C1_SlaveTransmit(const uint8_t *data, uint32_t num) { return ARM_I2Cx_SlaveTransmit(&I2C1_res, data, num);}
static int32_t ARM_I2C1_SlaveReceive(uint8_t *data, uint32_t num) { return ARM_I2Cx_SlaveReceive(&I2C1_res, data, num);}
static int32_t ARM_I2C1_GetDataCount(void) { return ARM_I2Cx_GetDataCount(&I2C1_res);}
static int32_t ARM_I2C1_Control(uint32_t control, uint32_t arg) { return ARM_I2Cx_Control(&I2C1_res, control, arg);}
static ARM_I2C_STATUS ARM_I2C1_GetStatus(void) { return ARM_I2Cx_GetStatus(&I2C1_res);}
void TWI1_Handler(void) { ARM_I2Cx_Handler(&I2C1_res);}

ARM_DRIVER_I2C Driver_I2C1 =
{
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    ARM_I2C1_Initialize,
    ARM_I2C1_Uninitialize,
    ARM_I2C1_PowerControl,
    ARM_I2C1_MasterTransmit,
    ARM_I2C1_MasterReceive,
    ARM_I2C1_SlaveTransmit,
    ARM_I2C1_SlaveReceive,
    ARM_I2C1_GetDataCount,
    ARM_I2C1_Control,
    ARM_I2C1_GetStatus
};
#endif


ARM_DRIVER_VERSION ARM_I2C_GetVersion(void)
{
    return DriverVersion;
}

ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

int32_t ARM_I2Cx_Initialize(I2C_Resource *res, ARM_I2C_SignalEvent_t cb_event)
{
    if ((res->info->flags & I2C_FLAG_INITIALIZED) == 0)
    {
        Pio *pio = res->pins.pio;
        const uint32_t pins = (res->pins.pinSDA | res->pins.pinSCL);

        //Connect pins to peripheral A
        pio->PIO_ABCDSR[0] &= ~pins;
        pio->PIO_ABCDSR[1] &= ~pins;

        //Set pin control to peripheral
        pio->PIO_PDR = pins;

        //Register event
        res->info->event = cb_event;

        res->info->flags = I2C_FLAG_INITIALIZED;
    }

    return ARM_DRIVER_OK;
}

int32_t ARM_I2Cx_Uninitialize(I2C_Resource *res)
{
    if (res->info->flags & I2C_FLAG_INITIALIZED)
    {
        if (res->info->flags & I2C_FLAG_POWERED)
            ARM_I2Cx_PowerControl(res, ARM_POWER_OFF);

        //Return control to pio controller
        res->pins.pio->PIO_PER = (res->pins.pinSDA | res->pins.pinSCL);

        //Reset flags
        res->info->flags = 0;
    }

    return ARM_DRIVER_OK;
}

int32_t ARM_I2Cx_PowerControl(I2C_Resource *res, ARM_POWER_STATE state)
{
    //Check if driver is initialized
    if ((res->info->flags & I2C_FLAG_INITIALIZED) == 0)
        return ARM_DRIVER_ERROR;

    switch (state)
    {
    case ARM_POWER_OFF:
        if (res->info->flags & I2C_FLAG_POWERED)
        {
            //Disable IRQ
            NVIC_DisableIRQ(res->irqNum);

            //Disable RX and TX DMA channels
            res->twi->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;

            //Disable peripheral clock
            PMC->PMC_PCDR0 = (1 << res->irqNum);

            res->info->flags &= ~(I2C_FLAG_POWERED);
        }
        break;

    case ARM_POWER_FULL:
        if ((res->info->flags & I2C_FLAG_POWERED) == 0)
        {
            //Enable peripheral clock
            PMC->PMC_PCER0 = (1 << res->irqNum);

            //Reset device
            res->twi->TWI_CR = TWI_CR_SWRST;

            //Enable IRQ
            NVIC_ClearPendingIRQ(res->irqNum);
            NVIC_EnableIRQ(res->irqNum);

            res->info->flags |= I2C_FLAG_POWERED;
        }
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t ARM_I2Cx_MasterTransmit(I2C_Resource *res, uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{

}

int32_t ARM_I2Cx_MasterReceive(I2C_Resource *res, uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
}

int32_t ARM_I2Cx_SlaveTransmit(I2C_Resource *res, const uint8_t *data, uint32_t num)
{
}

int32_t ARM_I2Cx_SlaveReceive(I2C_Resource *res, uint8_t *data, uint32_t num)
{
}

int32_t ARM_I2Cx_GetDataCount(I2C_Resource *res)
{
}

int32_t ARM_I2Cx_Control(I2C_Resource *res, uint32_t control, uint32_t arg)
{
    switch (control)
    {
    case ARM_I2C_OWN_ADDRESS:
        if (arg & (ARM_I2C_ADDRESS_10BIT || ARM_I2C_ADDRESS_GC))
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        res->twi->TWI_SMR = TWI_SMR_SADR(arg);
        break;

    case ARM_I2C_BUS_SPEED:
        switch (arg)
        {
        case ARM_I2C_BUS_SPEED_STANDARD:
            {
                const uint32_t value = SystemCoreClock / (2 * 100000) - 4;
                res->twi->TWI_CWGR = TWI_CWGR_CHDIV(value) | TWI_CWGR_CLDIV(value);
            }
            break;

        case ARM_I2C_BUS_SPEED_FAST:
            {
                const uint32_t value = SystemCoreClock / (2 * 400000) - 4;
                res->twi->TWI_CWGR = TWI_CWGR_CHDIV(value) | TWI_CWGR_CLDIV(value);
            }
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        break;

    case ARM_I2C_BUS_CLEAR:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

    case ARM_I2C_ABORT_TRANSFER:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

ARM_I2C_STATUS ARM_I2Cx_GetStatus(I2C_Resource *res)
{
    return res->info->status;
}

void ARM_I2Cx_Handler(I2C_Resource *res)
{

}


