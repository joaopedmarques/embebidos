

/*
 * File:   LCD.h
 * Author:
 * Comments:
 * Revision history:
 */

/* Program Description: This program header provides routines
for controlling
* a STD HITACHI HD44780 and compatible LCDs
*
* Hardware Description:
*
* RS ---> RB0
* R/W ---> GND
* EN ---> PF3
* D4 ---> RB5
* D5 ---> RB2
* D6 ---> RB4
* D7 ---> RB3
*
*/

/**************************************************************
*Includes and defines
**************************************************************/
// STD XC8 include
#include <inc/tm4c123gh6pm.h>
//#include "mcc_generated_files/mcc.h"
#include <stdlib.h>
#include <math.h>
//#define RS GPIO_PIN_2 //Register Select (Character or Instruction)
//#define EN GPIO_PIN_3 //LCD Clock Enable PIN, Falling Edge Triggered
// 4 bit operation
//#define D4 GPIO_PIN_4 //Bit 4
//#define D5 GPIO_PIN_5 //Bit 5
//#define D6 GPIO_PIN_6 //Bit 6
//#define D7 GPIO_PIN_7 //Bit 7
// function prototypes
uint32_t MENUTaskInit(void);
static void MENUTask(void);
