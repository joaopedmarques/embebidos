//Includes
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "LCD.h"


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif


int
main(void)
{

       //
       // Setup the system clock to run at 50 Mhz from PLL with crystal reference
       //
       SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                       SYSCTL_OSC_MAIN);

       //
       // Enable and wait for the port to be ready for access
       //
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //lcd
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //Leds
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); //Keypad X
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); //Keypad y



    volatile uint32_t ui32Loop;

    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
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

    //
    // Loop forever.
    //
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
        Lcd_Write_Char('G');
        Lcd_Write_Char('R');
        Lcd_Write_Char('U');
        Lcd_Write_Char('P');
        Lcd_Write_Char('O');
        Lcd_Write_Char('5');

        //
        // Delay for a bit
        //
        SysCtlDelay(20000000);

        Lcd_Clear();

        //
        // Turn on/off the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0); //Red LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2); //Blue LED

        //
        // Delay for a bit.
        //
        for(ui32Loop = 0; ui32Loop < 1000000; ui32Loop++)
        {
        }
    }
}
