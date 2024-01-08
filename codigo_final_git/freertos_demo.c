#include <I2C_task.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "LCD.h"
#include "menu_task.h"
#include "keypad.h"
#include "uart_task.h"
#include "PWM_task.h"

xQueueHandle g_pKEYQueue;
xQueueHandle g_pTempQueue;
xQueueHandle g_pLCDQueue;
xQueueHandle g_pUARTQueue;

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Leds
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);//Green LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); // Blue LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);//RED LED
    //
    // Set the clocking to run at 50 MHz from the PLL.
    //


    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Cria as Queue do sistema
    //
    g_pKEYQueue = xQueueCreate(48, sizeof(char*));
    g_pTempQueue = xQueueCreate(32, sizeof(char*));
    g_pLCDQueue = xQueueCreate(32, sizeof(char*));
    g_pUARTQueue = xQueueCreate(16,sizeof(char*));


    //
    // Create the LED task
    //
   /* if(LEDTaskInit() != 0)
    {

        while(1)
        {

        }

    }*/

    //
    // Create the I2C task.
    //
    if(I2CTaskInit() != 0)
    {

        while(1)
        {
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }
    }

    //
    // Create the Key task.
    //


    if(KEYTaskInit() != 0)
    {

        while(1)
        {
        }
    }
    //
    // Create the LCD task.
    //
    if(LCDTaskInit() != 0)
    {

        while(1)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //it's on so it sucks...
        }
    }
    //
    // Create the PWM task.
    //
    if(PWMTaskInit() != 0)
        {

            while(1)
            {
            }
        }

    if(UARTTaskInit() != 0)
    {

        while(1)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }
    }


    if(MENUTaskInit() != 0)
    {

        while(1)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        }
    }


    // Enable the NVIC
    IntMasterEnable();

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //turn on green led

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //

    while(1)
    {
    }
}
