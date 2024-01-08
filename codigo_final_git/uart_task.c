/*
 * uart_task.c
 *
 *  Created on: Jan 7, 2024
 *      Author: student
 */
#include <uart_task.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <math.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"

#define UARTTASKSTACKSIZE 500

xQueueHandle g_pUARTQueue;

void UARTIntHandler(void){

    uint32_t ui32Status;
    char uartrx;
    char buff[44];
    int i;

    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART5_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART5_BASE, ui32Status);


    //
    // Loop while there are characters in the receive FIFO.
    //
    i = 0;
    while(ROM_UARTCharsAvail(UART5_BASE))
    {


        //
        // Read the next character from the UART and write it back to the UART.
        //
        uartrx = ROM_UARTCharGetNonBlocking(UART5_BASE);

        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

        buff[i%44] = uartrx;
        i++;
        if (buff[0] != 'P'){
            i = 0;
        }

    }

    buff[0] = 'P';

    for (i = 0; i< 44; i++){
        xQueueSendToBack( g_pUARTQueue, &buff[i], 0 );
    }

    //
    // Clear the asserted interrupts.
    //
    //ROM_UARTIntClear(UART5_BASE, ui32Status);


}


uint32_t UARTTaskInit(void){

    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();


    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable(); // maybe later

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    ROM_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //enable port E interrupt handler keypad input
    UARTIntRegister(UART5_BASE, UARTIntHandler);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //9600
    ROM_UARTConfigSetExpClk(UART5_BASE, ROM_SysCtlClockGet(), 9600,
                               (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART5);
    ROM_UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);


    if(xTaskCreate(UARTTask, (const portCHAR *)"UART", UARTTASKSTACKSIZE, NULL,
                       tskIDLE_PRIORITY + PRIORITY_I2C_TASK, NULL) != pdTRUE)
        {
            return(1);
        }


    //
    // Success.
    //
    return(0);
}

static void UARTTask(void){

    while(1){

    }

}


