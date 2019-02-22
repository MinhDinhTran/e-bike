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

#include "project.h"

#define PWM_FREQUENCY 50000

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

uint32_t ui32ADC0Value[2];
const uint32_t vMax = 3625;  // 4.5 / 5 * 4096
const uint32_t vMin = 200;   // 0.5 / 5 * 4096

const uint32_t pwmBias = 500;
const uint32_t pwmMIN = 50;
const uint32_t pwmMAX = 950;

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui8Adjust;
volatile uint32_t sensedCurrent;
volatile uint32_t abc;
volatile bool a;
volatile bool b;
volatile bool c;

volatile bool switch1 = true;
volatile bool switch2 = true;
volatile bool isWithinCurrentBound = true;

int setAbc(bool a, bool b, bool c) {
  int abc = 0;
  if (a) abc = abc + 4;
  if (b) abc = abc + 2;
  if (c) abc = abc + 1;
  return abc;
}

void pwmControl(uint32_t gateH,uint32_t  gateH_other,uint32_t gateL, uint32_t gateOff1,uint32_t gateOff2,uint32_t gateOff3){
    setPulseWidth();
//  Enable duty cycle for top gate
    ROM_PWMOutputInvert(PWM0_BASE,gateH,true);
    ROM_PWMOutputState(PWM0_BASE,gateH,true);
//  Invert other gate of same leg
    ROM_PWMOutputInvert(PWM0_BASE,gateH_other,false);
    ROM_PWMOutputState(PWM0_BASE,gateH_other,true);

//  SC the return path
    ROM_PWMOutputInvert(PWM0_BASE,gateL,false);
    ROM_PWMOutputState(PWM0_BASE,gateL,false);
//  OC other 3 gates
    ROM_PWMOutputInvert(PWM0_BASE,gateOff1,true);
    ROM_PWMOutputState(PWM0_BASE,gateOff1,false);
    ROM_PWMOutputInvert(PWM0_BASE,gateOff2,true);
    ROM_PWMOutputState(PWM0_BASE,gateOff2,false);
    ROM_PWMOutputInvert(PWM0_BASE,gateOff3,true);
    ROM_PWMOutputState(PWM0_BASE,gateOff3,false);
}

void setPulseWidth(){

        ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                       ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,
                      ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,
                       ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5,
                      ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
                      ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,
                       ui8Adjust * ui32Load / 1000);
}

void turnOffPwm(){
    ROM_PWMOutputInvert(PWM0_BASE,PWM_OUT_6_BIT|PWM_OUT_7_BIT|PWM_OUT_5_BIT|PWM_OUT_2_BIT|PWM_OUT_3_BIT|PWM_OUT_4_BIT,true);
    ROM_PWMOutputState(PWM0_BASE,PWM_OUT_6_BIT|PWM_OUT_7_BIT|PWM_OUT_5_BIT|PWM_OUT_2_BIT|PWM_OUT_3_BIT|PWM_OUT_4_BIT,false);
}

void updateGates() {
  a = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) & GPIO_PIN_4;
  b = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) & GPIO_PIN_3;
  c = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) & GPIO_PIN_2;

  abc = setAbc(a, b, c);
  
  if (switch1 && isWithinCurrentBound) {
    switch (abc) {
      case 6: { // C high through and B Low return
        pwmControl(PWM_OUT_6_BIT,PWM_OUT_7_BIT,PWM_OUT_5_BIT,PWM_OUT_2_BIT,PWM_OUT_3_BIT,PWM_OUT_4_BIT);
        break;
      }
      case 4: { // A high through and B Low return
        pwmControl(PWM_OUT_2_BIT,PWM_OUT_3_BIT,PWM_OUT_5_BIT,PWM_OUT_6_BIT,PWM_OUT_7_BIT,PWM_OUT_4_BIT);
        break;
      }
      case 5: { // A High and C low
        pwmControl(PWM_OUT_2_BIT,PWM_OUT_3_BIT,PWM_OUT_7_BIT,PWM_OUT_6_BIT,PWM_OUT_5_BIT,PWM_OUT_4_BIT);
        break;
      }
      case 1: { // B high and C low
        pwmControl(PWM_OUT_4_BIT,PWM_OUT_5_BIT,PWM_OUT_7_BIT,PWM_OUT_2_BIT,PWM_OUT_3_BIT,PWM_OUT_6_BIT);
        break;
      }
      case 3: { // B high and A low
        pwmControl(PWM_OUT_4_BIT,PWM_OUT_5_BIT,PWM_OUT_3_BIT,PWM_OUT_2_BIT,PWM_OUT_7_BIT,PWM_OUT_6_BIT);
        break;
      }
      case 2: { // C high and A low
        pwmControl(PWM_OUT_6_BIT,PWM_OUT_7_BIT,PWM_OUT_3_BIT,PWM_OUT_2_BIT,PWM_OUT_4_BIT,PWM_OUT_5_BIT);
        break;
      }
      default: {
        turnOffPwm();
        break;
      }
    }
  } else {
    turnOffPwm();
  }
}

void configureBoard() {
  uint32_t ui32Period;
  ui8Adjust = pwmMIN;
  SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                 SYSCTL_OSC_MAIN);
  ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

  // Define Inputs
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);  // Hall A
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);  // Hall B
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);  // Hall C

  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);  // 3 Pos 1
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);  // 3 Pos 2

  // Define Outputs
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                        GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // LEDS

  // Configure PWM
  ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4); // AH
  ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5); // AL
  ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4); // BH
  ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5); // BL
  ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0); // CH
  ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1); // CL

  ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2);
  ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3);
  ROM_GPIOPinConfigure(GPIO_PE4_M0PWM4);
  ROM_GPIOPinConfigure(GPIO_PE5_M0PWM5);
  ROM_GPIOPinConfigure(GPIO_PD0_M0PWM6);
  ROM_GPIOPinConfigure(GPIO_PD1_M0PWM7);

  ui32PWMClock = SysCtlClockGet() / 2;
  ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);

  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load);

  ROM_PWMPulseWidthSet(
      PWM0_BASE,
      PWM_OUT_2 | PWM_OUT_3 | PWM_OUT_4 | PWM_OUT_5 | PWM_OUT_6 | PWM_OUT_7,
      pwmMIN * ui32Load / 1000);
  ROM_PWMOutputState(PWM0_BASE,
                     PWM_OUT_2_BIT | PWM_OUT_3_BIT | PWM_OUT_4_BIT |
                         PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT,
                     true);
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);

  // Configure ADC
  ADCHardwareOversampleConfigure(ADC0_BASE, 4);
  ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 1,
                           ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 0);

  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);  // Current Sensor
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);  // Throttle

  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  ui32Period = (SysCtlClockGet() / 10) / 2;
  TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
  TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
  // Interupt enable
  ADCIntRegister(ADC0_BASE, 0, ADC0IntHandler);
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
  GPIOIntRegister(GPIO_PORTA_BASE, GPIOIntHandler);
  GPIOIntRegister(GPIO_PORTB_BASE, GPIOBIntHandler);

  GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2,
                 GPIO_BOTH_EDGES);

  GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_2, GPIO_BOTH_EDGES);

  GPIOIntEnable(GPIO_PORTA_BASE,
                GPIO_INT_PIN_4 | GPIO_INT_PIN_3 | GPIO_INT_PIN_2);
  GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_3 | GPIO_INT_PIN_2);

  IntEnable(INT_TIMER0A);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_ADC0SS0);
  ADCIntEnable(ADC0_BASE, 0);

  IntMasterEnable();
  TimerEnable(TIMER0_BASE, TIMER_A);
}

int main(void) {
  configureBoard();
  updateGates();
  while (1) {
  }
}

void ADC0IntHandler(void) {
  ADCIntClear(ADC0_BASE, 0);

  ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value);

  sensedCurrent = ui32ADC0Value[0];
  if (sensedCurrent > vMax || sensedCurrent < vMin) {
    isWithinCurrentBound = false;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
  } else {
    isWithinCurrentBound = true;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
  }

  ui8Adjust = (ui32ADC0Value[1] - pwmBias) * 1000 / (4096 - pwmBias);
  if (ui8Adjust < pwmMIN) {
    ui8Adjust = pwmMIN;
  } else if (ui8Adjust > pwmMAX) {
    ui8Adjust = pwmMAX;
  }
  updateGates();
}

void GPIOIntHandler(void) {
  GPIOIntClear(GPIO_PORTA_BASE,
               GPIO_INT_PIN_4 | GPIO_INT_PIN_3 | GPIO_INT_PIN_2);
  updateGates();
}

void GPIOBIntHandler(void) {
  GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_3 | GPIO_INT_PIN_2);
  switch2 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3) & GPIO_PIN_3;
  switch1 = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2) & GPIO_PIN_2;
  updateGates();
}

void Timer0IntHandler(void) { TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); }
