/*
 * keyp.c
 *
 *  Created on: Jan 5, 2024
 *      Author: student
 */

#include <I2C_task.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "drivers/rgb.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "LCD.h"


//*****************************************************************************
//
// The stack size for the KEY toggle task.
//
//*****************************************************************************
#define KEYTASKSTACKSIZE        3000         // Stack size in words

//*****************************************************************************
//
// The queue that holds messages sent to the KEY task.
//
//*****************************************************************************

xQueueHandle g_pKEYQueue;

//codigo interrupt
void PortBIntHandler(void)
{
    // Handle the interrupt here

            //Interrupt
            //Lcd_Clear();
            //Lcd_Write_Char('X');
portBASE_TYPE xTaskWokenByRecieve;
int8_t linha1, linha2, linha3, linha4;
char tecla = 'Z';
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //turn on green led
    vTaskDelay(10 / portTICK_RATE_MS);

    GPIOPinWrite(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7), GPIO_PIN_4);
        linha1 = GPIOPinRead(GPIO_PORTB_BASE,(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
        if (linha1 == 0x01){
            tecla = '1';

        }
        else if (linha1 == 0x02){
            tecla = '2';
        }
        else if (linha1 == 0x04){
            tecla = '3';
        }
        else if (linha1 == 0x08){
            tecla = 'F';
        }

        //Lcd_Clear();
        //xQueueSendToBackFromISR(g_pKEYQueue, (void*)tecla, NULL);
        //Lcd_Write_Char(tecla);


    GPIOPinWrite(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7), GPIO_PIN_5);
        linha2 = GPIOPinRead(GPIO_PORTB_BASE,(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
        if (linha2 == 0x01){
            tecla = '4';
        }
        else if (linha2 == 0x02){
            tecla = '5';
        }
        else if (linha2 == 0x04){
            tecla = '6';
        }
        else if (linha2 == 0x08){
            tecla = 'E';
        }

        //Lcd_Clear();
        //xQueueSendToBackFromISR(g_pKEYQueue, (void*)tecla, NULL);
        //Lcd_Write_Char(tecla);

    GPIOPinWrite(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7), GPIO_PIN_6);
        linha3 = GPIOPinRead(GPIO_PORTB_BASE,(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
        if (linha3 == 0x01){
            tecla = '7';
        }
        else if (linha3 == 0x02){
            tecla = '8';
        }
        else if (linha3 == 0x04){
            tecla = '9';
        }
        else if (linha3 == 0x08){
            tecla = 'D';
        }

        //Lcd_Clear();
        //xQueueSendToBackFromISR(g_pKEYQueue, (void*)tecla, NULL);
        //Lcd_Write_Char(tecla);

    GPIOPinWrite(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7), GPIO_PIN_7);
        linha4 = GPIOPinRead(GPIO_PORTB_BASE,(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
        if (linha4 == 0x01){
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //turn off green led
            tecla = 'A';
        }
        else if (linha4 == 0x02){
            tecla = '0';

        }
        else if (linha4 == 0x04){
            tecla = 'B';
        }
        else if (linha4 == 0x08){
            tecla = 'C';
        }

       // Lcd_Clear();

        xQueueSendToBackFromISR(g_pKEYQueue, &tecla, xTaskWokenByRecieve);

        //Lcd_Write_Char(tecla);


        //vTaskDelay(1 / portTICK_RATE_MS);


            GPIOIntClear(GPIO_PORTB_BASE, GPIOIntStatus(GPIO_PORTB_BASE, true));

    //meter flag que faz entrar num if na main e ler
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0); //turn off green led
}


static void
KEYTask()
{

    SysTickEnable();
    while(1)
    {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4); //Todos os pins Y do keypad a 1 para ver se alguma tecla é premida
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);

    }
}


//*****************************************************************************
//
// Initializes the Key task.
//
//*****************************************************************************
uint32_t
KEYTaskInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Keypad X
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //Keypad y

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    //enable port B interrupt handler keypad input
    GPIOIntRegister(GPIO_PORTB_BASE, PortBIntHandler);

    //
    // Make pin 5 high level triggered interrupts.
    //
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_RISING_EDGE); // GPIO_HIGH_LEVEL
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_RISING_EDGE);

    // Enable interrupts on PB
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);




    //
    // Create the KEY task.
    //
    if(xTaskCreate(KEYTask, (const portCHAR *)"KEY", KEYTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_KEY_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
