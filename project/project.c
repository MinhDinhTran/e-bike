//*****************************************************************************
//
// project.c - Simple project to use as a starting point for more complex
//             projects.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

#include "driverlib/debug.h"
#include "driverlib/adc.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Simple Project (project)</h1>
//!
//! A very simple example that can be used as a starting point for more complex
//! projects.  Most notably, this project is fully TI BSD licensed, so any and
//! all of the code (including the startup code) can be used as allowed by that
//! license.
//!
//! The provided code simply toggles a GPIO using the Tiva Peripheral Driver
//! Library.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Toggle a GPIO.
//
//*****************************************************************************

int setAbc(bool a, bool b, bool c) {
    int abc = 0;
        if(a) abc = abc + 4;
        if(b) abc = abc + 2;
        if(c) abc = abc + 1;
        return abc;
}

void initTimer(){
uint32_t ui32Period;
SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
ui32Period = (SysCtlClockGet() / 10) / 2;
TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
IntEnable(INT_TIMER0A);
TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
IntMasterEnable();
TimerEnable(TIMER0_BASE, TIMER_A);
}

void pins(){

      volatile uint32_t abc;
        volatile bool a;
        volatile bool b;
        volatile bool c;

       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) { }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) { }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) { }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) { }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) { }

        // Define Inputs
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4); // Hall A
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3); // Hall B
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2); // Hall C

        GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_1); // Current Sensor A
        GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2); // Current Sensor B
        GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_3); // Current Sensor C


        // Define Outputs
        GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4); // A up - 1
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4); // A lo - 2
       GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0); // B up - 3
        GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7); // B lo - 4
        GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4); // C up - 5
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5); // C lo - 6


    // Loop forever.
    while(1){

            a = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) & GPIO_PIN_4;
            b = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) & GPIO_PIN_3;
            c = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) & GPIO_PIN_2;

            abc = setAbc(a,b,c);

            switch (abc) {
                case 6: {
//                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_PIN_4 | GPIO_PIN_0);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x0);
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
                    break;
                }
                case 4: {
 //                   GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_0, 0x0 | GPIO_PIN_0);

                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
                    break;
                }
                case 5: {
   //                 GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_0, 0x0 | GPIO_PIN_0);

                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x0);
                    break;
                }
                case 1: {
//                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_PIN_4 | 0x0);

                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0x0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x0);

                    break;
                }
                case 3: {
 //                   GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_PIN_4 | 0x0);

                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x0);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0x0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
                    break;
                }
                case 2: {
//                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_PIN_4 | GPIO_PIN_0);

                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x0);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x0);
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
                    break;
                }
                default: {
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
                    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
                    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
                    break;
                }
            }
    }

}

int main(void){
        initTimer();
        pins();
}

void Timer0IntHandler(void) {
// Clear the timer interrupt
TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
// Read the current state of the GPIO pin and
// write back the opposite state
if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)){
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
} else {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
}
}
