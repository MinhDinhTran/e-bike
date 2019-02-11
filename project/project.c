#include <stdbool.h>
#include <stdint.h>
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#define PWM_FREQUENCY 50000

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

uint32_t ui32ADC0Value[4];
const uint32_t vMax = 4000;
const uint32_t vMin = 600;
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui8Adjust;

void ADC0IntHandler(void) {
  ADCIntClear(ADC0_BASE, 0);

  ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value);

  if (ui32ADC0Value[0] > vMax || ui32ADC0Value[0] < vMin ||
      ui32ADC0Value[1] > vMax || ui32ADC0Value[1] < vMin ||
      ui32ADC0Value[2] > vMax || ui32ADC0Value[2] < vMin) {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
  } else {
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
  }



  ui8Adjust = ui32ADC0Value[3] % 4096;
  if(ui8Adjust < 1100){
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 3 * ui32Load/1000);
  } else if(ui8Adjust > 4000){
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Load);
  } else {
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (ui8Adjust-1100) * ui32Load/2996);
  }
}

void Timer0IntHandler(void) { TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); }

void initTimer() {
  uint32_t ui32Period;
  ui8Adjust = 500;
  SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                 SYSCTL_OSC_MAIN);
  ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

  // Define Inputs
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);  // Hall A
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);  // Hall B
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);  // Hall C
  // Define Outputs
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);  // A up - 1
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);  // A lo - 2
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);  // B up - 3
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);  // B lo - 4
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);  // C up - 5
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);  // C lo - 6
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // LEDS

  // Configure PWM
  ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
  ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0);
  ui32PWMClock = SysCtlClockGet() / 2;
  ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
  PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
  ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
  ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
  // Configure ADC
  ADCHardwareOversampleConfigure(ADC0_BASE, 4);
  ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 3,
                           ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 0);

  GPIOPinTypeADC(
      GPIO_PORTE_BASE,
      GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // 1: Current Sensor A 2:  Current
                                              // Sensor B 3:  Current Sensor C
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);  // Throttle

  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  ui32Period = (SysCtlClockGet() / 10) / 2;
  TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
  TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

  ADCIntRegister(ADC0_BASE, 0, ADC0IntHandler);
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);

  IntEnable(INT_TIMER0A);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_ADC0SS0);
  ADCIntEnable(ADC0_BASE, 0);

  IntMasterEnable();
  TimerEnable(TIMER0_BASE, TIMER_A);
}

int setAbc(bool a, bool b, bool c) {
  int abc = 0;
  if (a) abc = abc + 4;
  if (b) abc = abc + 2;
  if (c) abc = abc + 1;
  return abc;
}

void writeToGates(bool AH, bool AL, bool BH, bool BL, bool CH, bool CL) {
  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, (AH) ? GPIO_PIN_4 : 0x0);
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, (AL) ? GPIO_PIN_4 : 0x0);
  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, (BH) ? GPIO_PIN_0 : 0x0);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, (BL) ? GPIO_PIN_7 : 0x0);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, (CH) ? GPIO_PIN_4 : 0x0);
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, (CL) ? GPIO_PIN_5 : 0x0);
}

void pins() {
  volatile uint32_t abc;
  volatile bool a;
  volatile bool b;
  volatile bool c;

  // Loop forever.
  while (1) {
    a = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) & GPIO_PIN_4;
    b = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) & GPIO_PIN_3;
    c = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) & GPIO_PIN_2;

    abc = setAbc(a, b, c);

    switch (abc) {
      case 6: {
        writeToGates(true, true, true, false, false, true);
        break;
      }
      case 4: {
        writeToGates(false, true, true, false, true, true);
        break;
      }
      case 5: {
        writeToGates(false, true, true, true, true, false);
        break;
      }
      case 1: {
        writeToGates(true, true, false, true, true, false);
        break;
      }
      case 3: {
        writeToGates(true, false, false, true, true, true);
        break;
      }
      case 2: {
        writeToGates(true, false, true, true, false, true);
        break;
      }
      default: {
        writeToGates(true, true, true, true, true, true);
        break;
      }
    }
  }
}

int main(void) {
  initTimer();
  pins();
}
