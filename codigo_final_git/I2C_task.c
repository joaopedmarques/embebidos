#include <I2C_task.h>
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

//*****************************************************************************
//
// The stack size for the I2C toggle task.
//
//*****************************************************************************
#define I2CTASKSTACKSIZE        400         // Stack size in words

//*****************************************************************************
//
// The queue that holds messages sent to the I2C task.
//
//*****************************************************************************
xQueueHandle g_pTempQueue;

//#define SLAVE_ADDRESS_WRITE 0x48
//#define SLAVE_ADDRESS_READ 0x48
//#define CONFIG_TMP 0x01
//#define CONFIG_TMP_BITS 0x00
//#define TEMP_REG 0x00
#define I2C_TMP_ADDR 0x48 // TMP 100
#define I2C_TEMP_REG 0x00
#define I2C_CONFIG_REG 0x01
#define I2C_CONFIG_1 0x20


//*****************************************************************************
//
// Usando o protocolo de I2C essa tarefa adquire os valores da temperatura
// do sensor e repassa para PWM atraves de uma queue
//
//*****************************************************************************

static void
I2CSENDCONFIG(){
   // I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDRESS_WRITE , false);
    //I2CMasterDataPut(I2C3_BASE, CONFIG_TMP);
    //I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    //while(I2CMasterBusy(I2C3_BASE));
    //I2CMasterDataPut(I2C3_BASE, CONFIG_TMP_BITS);
    //I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH );
    //while(I2CMasterBusy(I2C3_BASE));

    // I2C configure
    I2CMasterSlaveAddrSet(I2C3_BASE, I2C_TMP_ADDR, false); // write

    I2CMasterDataPut(I2C3_BASE, I2C_CONFIG_REG); //01?
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C3_BASE));  //we add bus it works


    I2CMasterDataPut(I2C3_BASE, I2C_CONFIG_1); //why?

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C3_BASE));

}
float
I2CReceive()
{


    float temp;
    uint8_t temp1, temp2;
    float ms,m1;

    //I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS_WRITE, false);

    I2CMasterSlaveAddrSet(I2C3_BASE, I2C_TMP_ADDR, false); // write

    //I2CMasterDataPut(I2C1_BASE, TEMP_REG);
    //I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    //while(I2CMasterBusy(I2C1_BASE)); // delay de 40 ms
    // vTaskDelay(100 / portTICK_RATE_MS);

    //I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS_READ, true);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    I2CMasterDataPut(I2C3_BASE, I2C_TEMP_REG);
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);  //erro esta neste single read
    while(I2CMasterBusy(I2C3_BASE));
    //vTaskDelay(100 / portTICK_RATE_MS);

    I2CMasterSlaveAddrSet(I2C3_BASE, I2C_TMP_ADDR, true); // read

    //I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    //while(I2CMasterBusy(I2C1_BASE));
    while(I2CMasterBusy(I2C3_BASE));
    //vTaskDelay(1 / portTICK_RATE_MS);
    //temp1 = I2CMasterDataGet(I2C1_BASE);
    temp1 = I2CMasterDataGet(I2C3_BASE);
    ms = temp1;

    //I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    //I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    //while(I2CMasterBusy(I2C1_BASE));
    while(I2CMasterBusy(I2C3_BASE));
    //temp2 = I2CMasterDataGet(I2C1_BASE);
    temp2 = I2CMasterDataGet(I2C3_BASE);
    //vTaskDelay(100 / portTICK_RATE_MS);
    m1 = temp2;
    //I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    //while(I2CMasterBusy(I2C3_BASE));
    //temp3 = I2CMasterDataGet(I2C3_BASE);

    temp = ((temp1<<1)|(temp2>>8))*128/255;

    return temp;
}

// Reverses a string 'str' of length 'len'
void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}


void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

static void
I2CTask()
{
    char temperature[20];
    float uValue_Temp_New,uValue_Temp_Old;
    uValue_Temp_Old = 0;

    while(1)
    {
        vTaskDelay(150 / portTICK_RATE_MS);
        //uValue_Temp_New = I2CReceive(SLAVE_ADDRESS_READ);

        uValue_Temp_New = I2CReceive();
        ftoa(uValue_Temp_New, temperature, 2);
        while (uValue_Temp_New != uValue_Temp_Old)
        {
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); //turn on green led
            uValue_Temp_Old = uValue_Temp_New;
            //xQueueSendToBack( g_pTempQueue, &uValue_Temp_New, 1000 / portTICK_RATE_MS );
            //xQueueSendToBack( g_pTempQueue, (void*) &temperature, 100/ portTICK_RATE_MS );
            xQueueSendToBack( g_pTempQueue, &temperature, 0 );
        }
    }
}

//*****************************************************************************
//
// Initializes the I2C task.
//
//*****************************************************************************
uint32_t
I2CTaskInit(void)
{

        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

        //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Leds
        //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);//Green LED

        // wait for I2C
            while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
            {

            }

        //

        //GPIOPinConfigure(GPIO_PA6_I2C1SCL);
        //GPIOPinConfigure(GPIO_PA7_I2C1SDA);

        GPIOPinConfigure(GPIO_PD0_I2C3SCL);
        GPIOPinConfigure(GPIO_PD1_I2C3SDA);


        GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0); // SCL
        GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1); // SDA


        //GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
        //GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

        I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);



        I2CSENDCONFIG();

        //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

        //g_pTempQueue = xQueueCreate(4, sizeof(uint32_t));
        g_pTempQueue = xQueueCreate(32, sizeof(char*));

    //
    // Create the I2C task.
    //
    if(xTaskCreate(I2CTask, (const portCHAR *)"I2C", I2CTASKSTACKSIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_I2C_TASK, NULL) != pdTRUE)
    {
        return(1);
    }


    //
    // Success.
    //
    return(0);
}
