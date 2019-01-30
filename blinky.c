#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

void Timer0IntHandler(void){
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	
	
}

int setAbc(bool a, bool b, bool c) {
	int abc = 0;
		if(a) abc = abc + 4;
		if(b) abc = abc + 2;
		if(c) abc = abc + 1;
		return abc;
}

void initTimer(){
	    //
    // Set the clocking to run directly from the crystal.
    //
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet());

    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	
}

int main(void){

		//initTimer(); TODO develop interupts and PWM
	
	  volatile uint32_t abc;
		volatile bool a;
		volatile bool b;
		volatile bool c;
   
    // Enable the GPIO ports that are used for the digital input and output.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);


    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) { }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) { }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) { }

		// Define Inputs
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
		
		// Define Outputs
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
		GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
		GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
		GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
		
		GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
		GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);

    // Loop forever.
    while(1){
        
			a = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) & GPIO_PIN_4;
			b = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) & GPIO_PIN_3;
      c = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) & GPIO_PIN_2;
			
			abc = setAbc(a,b,c);
			
			switch (abc) {
				case 5: {
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x0);	
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x0);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
					break;
				}
				case 4: {
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x0);	
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
					break;
				}		
				case 6: {
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);	
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x0);
					break;
				}		
				case 2: {
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);	
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x0);
					break;
				}		
				case 3: {
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);	
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
					break;
				}		
				case 1: {
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);	
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x0);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
					break;
				}		
				default: {
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x0);
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x0);	
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x0);
					GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x0);
					break;
				}
			}
    }
}
