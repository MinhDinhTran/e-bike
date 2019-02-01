#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

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

int main(void){
	
		initTimer();
    // Enable the GPIO ports that are used for the digital input and output.
		while(1){
			
		}
}

void Timer0IntHandler(void) {
// Clear the timer interrupt
TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
// Read the current state of the GPIO pin and
// write back the opposite state
if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
{
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
}
else
{
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
}
}
