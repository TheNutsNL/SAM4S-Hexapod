#include "SDCard.h"

#include "FreeRTOS.h"
#include "task.h"

#define SDC_VERSION_1X
#define SDC_VERSION_200

#define SDC_OCR_2V7_2V8  (1 << 15)
#define SDC_OCR_2V8_2V9  (1 << 16)
#define SDC_OCR_2V9_3V0  (1 << 17)
#define SDC_OCR_3V0_3V1  (1 << 18)
#define SDC_OCR_3V1_3V2  (1 << 19)
#define SDC_OCR_3V2_3V3  (1 << 20)
#define SDC_OCR_3V3_3V4  (1 << 21)
#define SDC_OCR_3V4_3V5  (1 << 22)
#define SDC_OCR_3V5_3V6  (1 << 23)
#define SDC_OCR_S18      (1 << 24)
#define SDC_OCR_XPC      (1 << 28)
#define SDC_OCR_FB       (1 << 29)
#define SDC_OCR_HCS      (1 << 30)
#define SDC_OCR_BUSY     (1 << 31)

#define SDC_OCR_2V7_3V6  (SDC_OCR_2V7_2V8 | SDC_OCR_2V8_2V9 | SDC_OCR_2V9_3V0 | \
                          SDC_OCR_3V0_3V1 | SDC_OCR_3V1_3V2 | SDC_OCR_3V2_3V3 | \
                          SDC_OCR_3V3_3V4 | SDC_OCR_3V4_3V5 | SDC_OCR_3V5_3V6)

#define SDC_FLAG_INITIALIZED    (1 << 0)
#define SDC_FLAG_HIGH_CAPACITY  (1 << 1)

typedef struct
{
    uint32_t cmd;
    uint32_t arg;
    uint32_t flags;
    uint32_t response[4];
    uint32_t result;
    TickType_t ticksToWait;
} CommandInfo_t;

static TaskHandle_t tskMCI;
static uint32_t SDC_flags = 0;

static void SDC_SendCMD(ARM_DRIVER_MCI *mci, CommandInfo_t *cmd);
static void SDC_SendACMD(ARM_DRIVER_MCI *mci, CommandInfo_t *cmd);

int32_t SDC_Initialize(ARM_DRIVER_MCI *mci)
{
    CommandInfo_t cmd;

    SDC_flags = 0;

    tskMCI = xTaskGetCurrentTaskHandle();

    //Set bus speed
    mci->Control(ARM_MCI_BUS_SPEED, 400000);

    //Initialize card
    cmd.cmd = 0;
    cmd.arg = 0;
    cmd.flags = ARM_MCI_RESPONSE_NONE | ARM_MCI_CARD_INITIALIZE;
    cmd.ticksToWait = pdMS_TO_TICKS(250);

    SDC_SendCMD(mci, &cmd);
    if (cmd.result != ARM_MCI_EVENT_COMMAND_COMPLETE)
        return -100;

    //Send command 0
    cmd.flags = ARM_MCI_RESPONSE_NONE;
    SDC_SendCMD(mci, &cmd);
    if (cmd.result != ARM_MCI_EVENT_COMMAND_COMPLETE)
        return -101;

    //Send command 8
    cmd.cmd = 8;
    cmd.arg = (1 << 8) | 0xAA;
    cmd.flags = ARM_MCI_RESPONSE_SHORT;

    SDC_SendCMD(mci, &cmd);
    if (cmd.result == ARM_MCI_EVENT_COMMAND_COMPLETE)
        cmd.arg = SDC_OCR_HCS;
    else
        cmd.arg = 0;

    //Send command ACMD41
    cmd.cmd = 41;
    cmd.arg |= SDC_OCR_2V7_3V6;

    do
    {
        SDC_SendACMD(mci, &cmd);
        if (cmd.result != ARM_MCI_EVENT_COMMAND_COMPLETE)
            return -41;
    } while (cmd.response[0] & SDC_OCR_BUSY);

    if (cmd.response[0] & SDC_OCR_HCS)
        SDC_flags |= SDC_FLAG_HIGH_CAPACITY;

    //Send command 2
    cmd.cmd = 2;
    cmd.arg = 0;
    cmd.flags = ARM_MCI_RESPONSE_LONG;

    SDC_SendCMD(mci, &cmd);
    if (cmd.result != ARM_MCI_EVENT_COMMAND_COMPLETE)
            return -2;

    //Request relative address (command 3)
    cmd.cmd = 3;
    cmd.flags = ARM_MCI_RESPONSE_LONG;

    SDC_SendCMD(mci, &cmd);
    if (cmd.result != ARM_MCI_EVENT_COMMAND_COMPLETE)
            return -2;

    SDC_flags |= SDC_FLAG_INITIALIZED;

    return 0;
}

void SDC_SendCMD(ARM_DRIVER_MCI *mci, CommandInfo_t *cmd)
{
    //Send command
    mci->SendCommand(cmd->cmd, cmd->arg, cmd->flags, cmd->response);

    //Wait for command to complete
    xTaskNotifyWait(0xFFFFFFFF, 0, &cmd->result, cmd->ticksToWait);
}

void SDC_SendACMD(ARM_DRIVER_MCI *mci, CommandInfo_t *cmd)
{
    CommandInfo_t cmd55;
    cmd55.cmd = 55;
    cmd55.arg = 0;
    cmd55.flags = ARM_MCI_RESPONSE_NONE;
    cmd55.ticksToWait = cmd->ticksToWait;

    SDC_SendCMD(mci, &cmd55);

    if (cmd55.result == ARM_MCI_EVENT_COMMAND_COMPLETE)
        SDC_SendCMD(mci, cmd);
    else
        cmd->result = cmd55.result;
}

void MCI_event (uint32_t event)
{
    BaseType_t taskWoken = pdFALSE;
    xTaskNotifyFromISR(tskMCI, event, eSetValueWithOverwrite, &taskWoken);
}
