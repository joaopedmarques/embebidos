/*
 * menu_task.c
 *
 *  Created on: Jan 6, 2024
 *      Author: student
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
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
#include "menu_task.h"
#include "LCD.h"

xQueueHandle g_pLCDQueue;
xQueueHandle g_pKEYQueue;
xQueueHandle g_pTempQueue;
xQueueHandle g_pUARTQueue;

#define MENUTASKSTACKSIZE        500


static void MENUTask(){

    char keypad_key = 'Z';
    char temp[20] = "";
    //char test_string[20] = "";
    char uart_buff[44] = "";
    char uart_lcd_buff[20];
    char uart_key = " ";
    portBASE_TYPE xStatus;
    int i;

    while(1){
        // recieve from keypad

        if(g_pKEYQueue != NULL){
            xStatus = xQueueReceive(g_pKEYQueue, &keypad_key, 0); // portMAX_DELAY
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
            if(xStatus == pdTRUE){
                xQueueSendToBack( g_pLCDQueue, &keypad_key, 0 );
            }
        }
        if(g_pTempQueue != NULL){
            xStatus = xQueueReceive( g_pTempQueue, &temp, 0); // portMAX_DELAY
            if( xStatus == pdTRUE ){
                xQueueSendToBack( g_pLCDQueue, &temp, 0 );
            }
        }
        if(g_pUARTQueue != NULL){
            i = 1;
            xStatus = xQueueReceive( g_pUARTQueue, &uart_key, 0);
            uart_buff[0] = uart_key;
            if (xStatus == pdTRUE){
                while (xStatus == pdTRUE){
                    xStatus = xQueueReceive( g_pUARTQueue, &uart_key, 0);
                    uart_buff[i] = uart_key;
                    i++;
                }
                //Lcd_Clear();
                for (i = 0;i < 20; i++){
                    uart_lcd_buff[i] = uart_buff[i];
                    uart_key = uart_buff[i];
                    xQueueSendToBack( g_pLCDQueue, &uart_key, 0);
                }
                //xQueueSendToBack( g_pLCDQueue, &uart_lcd_buff, 0);
            }
            /*
            xStatus = xQueueReceive( g_pUARTQueue, &uart_buff, 0); // portMAX_DELAY
            if( xStatus == pdTRUE ){
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                //Lcd_Clear();
                //Lcd_Write_Char('A');
                    for(i = 0; i< 20; i++){
                        uart_lcd_buff[i] = uart_buff[i];
                        Lcd_Write_String(uart_lcd_buff);
                    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
                    }
                //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
                //Lcd_Write_String(uart_buff);
                //xQueueSendToBack( g_pLCDQueue, &uart_buff, 0 );
            }*/
        }


        //xQueueSendToBack( g_pLCDQueue, &test_string, 0 );


    }

}

uint32_t
MENUTaskInit(void)
{


    //
    // Create the KEY task.
    //
    if(xTaskCreate(MENUTask, (const portCHAR *)"MENU", MENUTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_MEN_TASK, NULL) != pdPASS)
    {
        return(1);
    }
    //
    // Success.
    //
    return(0);
}
