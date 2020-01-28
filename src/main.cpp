/*
 * $projectname$.c
 *
 * Created: 06-05-2019
 * Author : Ashley Nuttall
 */


#include "sam.h"
//#include "Driver_USART.h"
//#include "Driver_MCI.h"
//#include "SDCard.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Driver_TWI.h"

#define LED_PIO PIOC
#define LED_PIN PIO_PC23

#define TASK_PRIORITY_BLINK     (tskIDLE_PRIORITY + 1)
#define TASK_PRIORITY_SERIAL    (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_SD_CARD   (tskIDLE_PRIORITY + 3)

using namespace System::Driver;

//extern ARM_DRIVER_MCI   Driver_MCI;
//extern ARM_DRIVER_USART Driver_USART1;
extern TWI Driver_TWI0;

volatile uint8_t isTXBusy;

static void Task_Blink(void *param);
//static void Task_Serial(void *param);

//void USART1_event (uint32_t event)
//{
//    switch (event)
//    {
//    case ARM_USART_EVENT_SEND_COMPLETE:
//        isTXBusy = 0;
//    }
//}


int main(void)
{
    SystemInit();

    Driver_TWI0.Initialize(0);

    xTaskCreate(Task_Blink, "Blink", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_BLINK, NULL);
    //xTaskCreate(Task_Serial, "Serial", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_SERIAL, NULL);

    vTaskStartScheduler();

    while(1);
}

void Task_Blink(void *param)
{
    //Enable led pin for output
    LED_PIO->PIO_PER = LED_PIN;
    LED_PIO->PIO_OER = LED_PIN;

    //Enable MCI driver
//    Driver_MCI.Initialize(MCI_event);
//    Driver_MCI.PowerControl(ARM_POWER_FULL);

//    SDC_Initialize(&Driver_MCI);

    while (1)
    {
//        if (Driver_MCI.ReadCD() == 1)
//            LED_PIO->PIO_CODR = LED_PIN;
//        else
//            LED_PIO->PIO_SODR = LED_PIN;

        vTaskDelay(500 / portTICK_PERIOD_MS);
        LED_PIO->PIO_SODR = LED_PIN;

        vTaskDelay(500 / portTICK_PERIOD_MS);
        LED_PIO->PIO_CODR = LED_PIN;
    }
}


//void Task_Serial(void *param)
//{
//    char data[] = "Hello world!\n";
//
//    Driver_USART1.Initialize(USART1_event);
//    Driver_USART1.PowerControl(ARM_POWER_FULL);
//    Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS, 125000);
//    Driver_USART1.Control(ARM_USART_CONTROL_RX, 1);
//    Driver_USART1.Control(ARM_USART_CONTROL_TX, 1);
//
//    while (1)
//    {
//        isTXBusy = 1;
//        Driver_USART1.Send(data, sizeof(data));
//
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }
//}

extern "C"
{
    void vApplicationIdleHook(void)
    {
        __WFI();
    }
    }


