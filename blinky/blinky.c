//Includes
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pwm.c"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include <math.h>
#include "LCD.h"

//#include "FreeRTOS.h"
//#include "task.h"

#define LED_PIN GPIO_PIN_0
#define LED_PIN_2 GPIO_PIN_1
#define BLINK_DELAY_MS 500
#define I2C_TMP_ADDR 0x48 // TMP 100
#define I2C_TEMP_REG 0x00
#define I2C_CONFIG_REG 0x01
#define I2C_CONFIG_1 0x20

/*
#include <I2C_task.h>
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
*/



//char tecla;


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif


//codigo interrupt
void PortBIntHandler(void)
{
    // Handle the interrupt here

            //Interrupt
            Lcd_Clear();
            Lcd_Write_Char('X');

int8_t linha1, linha2, linha3, linha4;
char tecla;

    GPIOPinWrite(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7), GPIO_PIN_4);
        linha1 = GPIOPinRead(GPIO_PORTB_BASE,(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
        if (linha1 == 0x01){
            tecla = '1';
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
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

        Lcd_Clear();
        Lcd_Write_Char(tecla);


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

        Lcd_Clear();
        Lcd_Write_Char(tecla);

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

        Lcd_Clear();
        Lcd_Write_Char(tecla);

    GPIOPinWrite(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6| GPIO_PIN_7), GPIO_PIN_7);
        linha4 = GPIOPinRead(GPIO_PORTB_BASE,(GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));
        if (linha4 == 0x01){
            tecla = 'A';
        }
        else if (linha4 == 0x02){
            tecla = '0';
            PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
        }
        else if (linha4 == 0x04){
            tecla = 'B';
        }
        else if (linha4 == 0x08){
            tecla = 'C';
        }

        Lcd_Clear();
        Lcd_Write_Char(tecla);


    SysCtlDelay(7000000);


            GPIOIntClear(GPIO_PORTB_BASE, GPIOIntStatus(GPIO_PORTB_BASE, true));

    //meter flag que faz entrar num if na main e ler
}
/*
void blink_task_1(void *pvParameters) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_PIN_1);

  while (1) {
    GPIOPinWrite(GPIO_PORTF_BASE, LED_PIN_1, LED_PIN_1);
    vTaskDelay(BLINK_DELAY_MS / portTICK_PERIOD_MS);
    GPIOPinWrite(GPIO_PORTF_BASE, LED_PIN_1, 0);
    vTaskDelay(BLINK_DELAY_MS / portTICK_PERIOD_MS);
  }
}

void blink_task_2(void *pvParameters) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_PIN_2);

  while (1) {
    GPIOPinWrite(GPIO_PORTF_BASE, LED_PIN_2, LED_PIN_2);
    vTaskDelay(BLINK_DELAY_MS / portTICK_PERIOD_MS);
    GPIOPinWrite(GPIO_PORTF_BASE, LED_PIN_2, 0);
    vTaskDelay(BLINK_DELAY_MS / portTICK_PERIOD_MS);
  }
}
*/

float i2c_get_temp (void){

    uint16_t aux = 0;
    float aux_sign = 0;

    I2CMasterSlaveAddrSet(I2C3_BASE, I2C_TMP_ADDR, false); // write

    I2CMasterDataPut(I2C3_BASE, I2C_TEMP_REG);
    I2CMasterDataPut(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusBusy(I2C3_BASE));

    I2CMasterSlaveAddrSet(I2C3_BASE, I2C_TMP_ADDR, true); // read

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusBusy(I2C3_BASE));
    aux = I2CMasterDataGet(I2C3_BASE);
    aux = aux << 8;

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusBusy(I2C3_BASE));
    aux = aux | I2CMasterDataGet(I2C3_BASE);

    if ((aux & 0x0800) == 0x0800){
        aux_sign = -128;
    }
    else {
        aux_sign = 0;
    }
    aux = aux & 0x07FF;

    return (aux_sign + ((float) aux) * 0.06250);
}

int main(void)
{

    uint16_t test;
    float temp;

       //
       // Setup the system clock to run at 50 Mhz from PLL with crystal reference
       //
       SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                       SYSCTL_OSC_MAIN);
       // PWM enable
       SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
       // I2C enable
       SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
       SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
       //
       // Enable and wait for the port to be ready for access
       //
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //lcd
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Leds
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Keypad X
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //Keypad y
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // I2C
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // DEBUG


    volatile uint32_t ui32Loop;


    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }
    // Wait for PWM
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }
    // wait for I2C
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
        {
        }

    // PWM pin
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
    // configure PWM
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //enable port B interrupt handler keypad input
    GPIOIntRegister(GPIO_PORTB_BASE, PortBIntHandler);


    // I2C pin setup
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0); // SCL
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1); // SDA

    // Initialize M and S
    I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), true);

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);//Green LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);//Blue LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);//Red LED
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, D4 | D5 | D6 | D7 | EN | RS); //pins LCD como output
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);   //definir pinos keypad Y como output    PB4,5,6,7
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);    //definir pinos keypad X como input     PC0,1,2,3

    //GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0); //DEBUG I2C
    //GPIO_PIN_WRITE



    Lcd_Init();

    //
    // Make pin 5 high level triggered interrupts.
    //
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_HIGH_LEVEL);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_HIGH_LEVEL);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_HIGH_LEVEL);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_HIGH_LEVEL);

    // Enable interrupts on PB
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);


    // configure pwm period and starting pulse width
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 20000);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 10000); // 0
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

/*
    while(1){

        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0b01);

    }*/

    // I2C configure
    I2CMasterSlaveAddrSet(I2C3_BASE, I2C_TMP_ADDR, false); // write

    I2CMasterDataPut(I2C3_BASE, I2C_CONFIG_REG); //01?
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusBusy(I2C3_BASE));  //we add bus it works but not on osciloscope


    I2CMasterDataPut(I2C3_BASE, I2C_CONFIG_1); //why?

    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusBusy(I2C3_BASE));



    //
    // Loop forever.
    //

    // Enable the NVIC
    IntMasterEnable();

    // PWM Output
    //PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);

    /*
    while(1){
      xTaskCreate(blink_task_1, "Blink Task 1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
      xTaskCreate(blink_task_2, "Blink Task 2", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
      vTaskStartScheduler();
    }*/

    while(1)
    {
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4); //Todos os pins Y do keypad a 1 para ver se alguma tecla é premida
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
        //
        // Turn on the LED. (Red)
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
        //
        // Write to the LCD
        //
        Lcd_Clear();
        SysCtlDelay(2000000);
        Lcd_Write_Char('G');
        Lcd_Write_Char('R');
        Lcd_Write_Char('U');
        Lcd_Write_Char('P');
        Lcd_Write_Char('O');
        Lcd_Write_Char('5');
        //PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);  //always on test
        //
        // Delay for a bit
        //
        SysCtlDelay(2000000);

        //Lcd_Clear();

        // Turn on/off the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0); //Red LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //Blue LED

        temp = i2c_get_temp();
        //READ I2C
        //
        /*I2CMasterSlaveAddrSet(I2C2_BASE, I2C_TMP_ADDR, false);  //write I2C

        I2CMasterDataPut(I2C2_BASE, I2C_TEMP_REG);
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        while(I2CMasterBusy(I2C2_BASE));

        I2CMasterSlaveAddrSet(I2C2_BASE, I2C_TMP_ADDR, true);  //read I2C

        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while(I2CMasterBusy(I2C2_BASE));
        test = I2CMasterDataGet(I2C2_BASE)<<8;

        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C2_BASE));
        test |= I2CMasterDataGet(I2C2_BASE); */



        // Delay for a bit.
        //
        SysCtlDelay(2000000);
    }
}
