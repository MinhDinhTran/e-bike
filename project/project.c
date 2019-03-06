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
#include "driverlib/fpu.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "pid.h"
#include "project.h"

#define PWM_FREQUENCY 50000

#define AH_TAG 0
#define AL_TAG 1
#define BH_TAG 2
#define BL_TAG 3
#define CH_TAG 4
#define CL_TAG 5

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

uint32_t ui32ADC0Value[2];
const uint32_t vMax = 3625;  // 4.5 / 5 * 4096
const uint32_t vMin = 200;   // 0.5 / 5 * 4096

const float iM = 25/2048;  // (actual current) = m(sensed value) + b
const float iB = -25;   // 2.5 / 5 * 4096

const uint32_t pwmBias = 500;
const uint32_t pwmMIN = 50;
const uint32_t pwmMAX = 950;
volatile uint32_t ui8Adjust;
// Current control variables
const float k_id = 27.1378;  // I gain i-d
const float k_pd = 0.0118;   // P gain i-d
float id_int = 0;
float id_dif = 0;
float id_err = 0;
// Speed control variables
const float k_is = 96331;  // I gain speed
const float k_ps = 3679.6;   // P gain speed
volatile float s_int = 0;
volatile float s_dif = 0;
volatile float s_err = 0;

volatile float sensedCurrentFloat = 0;
volatile float sensedSpeed = 0;
volatile float speedCommand = 0;

volatile float currentCommand = 0;
volatile float dutyCycle = 0;

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t throttle;
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

uint32_t getOutBit(uint32_t pin_value) {
  switch (pin_value) {
    case AH_TAG:
      return PWM_OUT_6_BIT;
    case AL_TAG:
      return PWM_OUT_7_BIT;
    case BH_TAG:
      return PWM_OUT_6_BIT;
    case BL_TAG:
      return PWM_OUT_7_BIT;
    case CH_TAG:
      return PWM_OUT_1_BIT;
    case CL_TAG:
      return PWM_OUT_0_BIT;
    default:
      return PWM_OUT_0_BIT;
  }
}

void pwmControl(uint32_t gateH, uint32_t gateH_other, uint32_t gateL,
                uint32_t gateOff1, uint32_t gateOff2, uint32_t gateOff3) {
  uint32_t gateHBase =
      (gateH == AH_TAG || gateH == AL_TAG) ? PWM1_BASE : PWM0_BASE;
  uint32_t gateLBase =
      (gateL == AH_TAG || gateL == AL_TAG) ? PWM1_BASE : PWM0_BASE;
  uint32_t otherBase =
      (gateOff2 == AH_TAG || gateOff2 == AL_TAG) ? PWM1_BASE : PWM0_BASE;

  uint32_t pinH = getOutBit(gateH);
  uint32_t pinH_O = getOutBit(gateH_other);
  uint32_t pinL = getOutBit(gateL);
  uint32_t pinL_O = getOutBit(gateOff1);
  uint32_t pinO = getOutBit(gateOff2);
  uint32_t pinO_O = getOutBit(gateOff3);

  setPulseWidth();

  //  Enable duty cycle for top gate
  ROM_PWMOutputInvert(gateHBase, pinH, true);
  ROM_PWMOutputState(gateHBase, pinH, true);
  //  Invert other gate of same leg
  ROM_PWMOutputInvert(gateHBase, pinH_O, false);
  ROM_PWMOutputState(gateHBase, pinH_O, true);

  //  SC the return path
  ROM_PWMOutputInvert(gateLBase, pinL, false);
  ROM_PWMOutputState(gateLBase, pinL, false);
  //  OC other 3 gates
  ROM_PWMOutputInvert(gateLBase, pinL_O, true);
  ROM_PWMOutputState(gateLBase, pinL_O, false);
  ROM_PWMOutputInvert(otherBase, pinO, true);
  ROM_PWMOutputState(otherBase, pinO, false);
  ROM_PWMOutputInvert(otherBase, pinO_O, true);
  ROM_PWMOutputState(otherBase, pinO_O, false);
}

void setPulseWidth() {
  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, ui8Adjust * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ui8Adjust * ui32Load / 1000);
}

void turnOffPwm() {
  ROM_PWMOutputInvert(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
  ROM_PWMOutputInvert(
      PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT,
      true);

  ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, false);
  ROM_PWMOutputState(
      PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_0_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT,
      false);
}

void updateGates() {
  a = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) & GPIO_PIN_4;
  b = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) & GPIO_PIN_3;
  c = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) & GPIO_PIN_2;

  abc = setAbc(a, b, c);

  if (switch1 && isWithinCurrentBound) {
    switch (abc) {
      case 6: {  // C high through and B Low return
        pwmControl(CH_TAG, CL_TAG, BL_TAG, BH_TAG, AH_TAG, AL_TAG);
        break;
      }
      case 4: {  // A high through and B Low return
        pwmControl(AH_TAG, AL_TAG, BL_TAG, BH_TAG, CH_TAG, CL_TAG);
        break;
      }
      case 5: {  // A High and C low
        pwmControl(AH_TAG, AL_TAG, CL_TAG, CH_TAG, BH_TAG, BL_TAG);
        break;
      }
      case 1: {  // B high and C low
        pwmControl(BH_TAG, BL_TAG, CL_TAG, CH_TAG, AH_TAG, AL_TAG);
        break;
      }
      case 3: {  // B high and A low
        pwmControl(BH_TAG, BL_TAG, AL_TAG, AH_TAG, CH_TAG, CL_TAG);
        break;
      }
      case 2: {  // C high and A low
        pwmControl(CH_TAG, CL_TAG, AL_TAG, AH_TAG, BH_TAG, BL_TAG);
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

  ROM_FPULazyStackingEnable();
  ROM_FPUEnable();

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
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

  // Define Inputs
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);  // Hall A
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);  // Hall B
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);  // Hall C

  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);  // 3 Pos 1
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);  // 3 Pos 2

  // Define Outputs
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);  // LEDS
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);  // LEDS
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);  // LEDS

  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
  // Configure PWM
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);  // AH
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);  // AL
  ROM_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);  // BH
  ROM_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);  // BL
  ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);  // CH
  ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);  // CL

  ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);
  ROM_GPIOPinConfigure(GPIO_PF3_M1PWM7);
  ROM_GPIOPinConfigure(GPIO_PC4_M0PWM6);
  ROM_GPIOPinConfigure(GPIO_PC5_M0PWM7);
  ROM_GPIOPinConfigure(GPIO_PB7_M0PWM1);
  ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);

  ui32PWMClock = SysCtlClockGet() / 2;
  ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

  PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);

  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);

  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6 | PWM_OUT_7,
                       pwmMIN * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0 | PWM_OUT_1 | PWM_OUT_6 | PWM_OUT_7,
                       pwmMIN * ui32Load / 1000);

  ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
  ROM_PWMOutputState(
      PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT,
      true);

  ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);

  // Configure ADC
  ADCHardwareOversampleConfigure(ADC0_BASE, 4);
  ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH2);
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

void checkCurrentLimit() {
  if (sensedCurrent > vMax || sensedCurrent < vMin) {
    isWithinCurrentBound = false;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
  } else {
    isWithinCurrentBound = true;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
  }
  updateGates();
}

void updatePWM(float dutyCycle){
  ui8Adjust = dutyCycle * 1000;
  if (ui8Adjust < pwmMIN) {
    ui8Adjust = pwmMIN;
  } else if (ui8Adjust > pwmMAX) {
    ui8Adjust = pwmMAX;
  }
  setPulseWidth();
}

void ADC0IntHandler(void) {
  ADCIntClear(ADC0_BASE, 0);

  ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value);

  sensedCurrent = ui32ADC0Value[0];
  throttle = ui32ADC0Value[1];
  checkCurrentLimit();

  sensedCurrentFloat = (sensedCurrent - 2048.0)/4096.0 * 50.0;

  //sensedCurrentFloat = 16.0;
  currentCommand = throttle/ 4096.0 * 15;
  dutyCycle = pidloop(currentCommand, sensedCurrentFloat, false, 
  k_pd, k_id, 0, 0.05, 0.95, 1.0/5000.0, &id_int, &id_err);
  updatePWM(dutyCycle);
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
