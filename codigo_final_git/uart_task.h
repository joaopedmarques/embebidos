/*
 * uart_task.h
 *
 *  Created on: Jan 7, 2024
 *      Author: student
 */
/**************************************************************
*Includes and defines
**************************************************************/
// STD XC8 include
#include <inc/tm4c123gh6pm.h>
//#include "mcc_generated_files/mcc.h"
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
// function prototypes
uint32_t UARTTaskInit(void);
static void UARTTask(void);
void UARTIntHandler(void);
