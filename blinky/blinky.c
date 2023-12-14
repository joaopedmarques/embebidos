//Includes
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pwm.c"
//#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "LCD.h"

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
/*



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

int
main(void)
{

       //
       // Setup the system clock to run at 50 Mhz from PLL with crystal reference
       //
       SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                       SYSCTL_OSC_MAIN);
       // PWM enable
       SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
       //
       // Enable and wait for the port to be ready for access
       //
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //lcd
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Leds
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Keypad X
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //Keypad y

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

    // PWM pin
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
    // configure PWM
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //enable port B interrupt handler keypad input
    GPIOIntRegister(GPIO_PORTB_BASE, PortBIntHandler);

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
    //
    // Loop forever.
    //

    // Enable the NVIC
    IntMasterEnable();

    // PWM Output
    //PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);


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

        // Delay for a bit.
        //
        SysCtlDelay(2000000);
    }
}
