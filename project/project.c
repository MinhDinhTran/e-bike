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

#define PWM_FREQUENCY 40000
#define TIMER_FREQUENCY 6000
#define DEBUG 1
#define PI 3.14159
#define TICK_PER_REV 138
#define SPEED_SENSOR_DELAY 4000

#define RADIUS .3302

#define AH_TAG 0
#define AL_TAG 1
#define BH_TAG 2
#define BL_TAG 3
#define CH_TAG 4
#define CL_TAG 5

#define k_id 0.1 //27.1378;  // I gain i-d
#define k_pd 0.02    // P gain i-d

#define k_is 0.45  // I gain speed
#define k_ps 0.7   // P gain speed

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {}
#endif

uint32_t ui32ADC0Value[7];
const uint32_t vMax = 3625;  // 4.5 / 5 * 4096
const uint32_t vMin = 200;   // 0.5 / 5 * 4096

const float is_Offset = 2080.0;  // 
const float is_Slope = 200.0;   

const uint32_t thrMIN = 660;
const uint32_t thrMAX = 3480;
const float cur_cmd_MAX = 10.0; // (A)

const float speed_cmd_MAX = 5.0; // (m/s)


const uint32_t pwmBias = 500;
const uint32_t pwmMIN = 20;
const uint32_t pwmMAX = 980;
volatile uint32_t pwmCurr = 50;
// Current control variables
float p_i_d_pos = k_pd + k_id/(2*TIMER_FREQUENCY);
float p_i_d_neg = - k_pd + k_id/(2*TIMER_FREQUENCY);

volatile float id_int = 0;
volatile float id_err = 0;
// Speed control variables

float p_i_i_pos = k_ps + k_is/(2*TIMER_FREQUENCY);
float p_i_i_neg = - k_ps + k_is/(2*TIMER_FREQUENCY);

volatile float s_int = 0;
volatile float s_err = 0;

volatile float sensedCurrentFloat = 0;
volatile float sensedSpeed = 0;
volatile float speedCommand = 0;

volatile uint32_t speedSensorCount = 0;

volatile float currentCommand = 0;
volatile float dutyCycle = 0;

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t throttle;

volatile uint32_t battery_voltage;
volatile uint32_t boost_voltage;
volatile uint32_t battery_current;

volatile uint32_t current_10[10];
volatile uint32_t currentIndex = 0;


volatile uint32_t sensedCurrent;
volatile uint32_t abc;
volatile bool a;
volatile bool b;
volatile bool c;

volatile uint32_t hallCount = 0;
volatile uint32_t maxCount = 0;

volatile bool switch1 = true;
volatile bool switch2 = true;
volatile bool isWithinCurrentBound = true;

volatile bool hallIsCounting = true;

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
      return PWM_OUT_4_BIT;
    case CL_TAG:
      return PWM_OUT_5_BIT;
    default:
      return PWM_OUT_0_BIT;
  }
}

uint32_t getBase(uint32_t gate){
   return (gate == AH_TAG || gate == AL_TAG) ? PWM1_BASE : PWM0_BASE;
}

void pwmControl(uint32_t gateH, uint32_t gateH_other, uint32_t gateL,
                uint32_t gateOff1, uint32_t gateOff2, uint32_t gateOff3) {
  uint32_t gateHBase = getBase(gateH);
  uint32_t gateHLBase = getBase(gateH_other);
  uint32_t gateLBase = getBase(gateL);
  uint32_t gateLLBase = getBase(gateOff1);
  uint32_t otherBase = getBase(gateOff2);
  uint32_t otherLBase = getBase(gateOff3);

  uint32_t pinH = getOutBit(gateH);
  uint32_t pinH_O = getOutBit(gateH_other);
  uint32_t pinL = getOutBit(gateL);
  uint32_t pinL_O = getOutBit(gateOff1);
  uint32_t pinO = getOutBit(gateOff2);
  uint32_t pinO_O = getOutBit(gateOff3);

  //  Enable duty cycle for top gate
  ROM_PWMOutputInvert(gateHBase, pinH, true);
  ROM_PWMOutputState(gateHBase, pinH, true);
  //  Invert other gate of same leg
  ROM_PWMOutputInvert(gateHLBase, pinH_O, false);
  ROM_PWMOutputState(gateHLBase, pinH_O, true);

  //  SC the return path
  ROM_PWMOutputInvert(gateLBase, pinL, false);
  ROM_PWMOutputState(gateLBase, pinL, false);
  //  OC other 3 gates
  ROM_PWMOutputInvert(gateLLBase, pinL_O, true);
  ROM_PWMOutputState(gateLLBase, pinL_O, false);
  ROM_PWMOutputInvert(otherBase, pinO, true);
  ROM_PWMOutputState(otherBase, pinO, false);
  ROM_PWMOutputInvert(otherLBase, pinO_O, true);
  ROM_PWMOutputState(otherLBase, pinO_O, false);
}

void setPulseWidth() {
  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, pwmCurr * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pwmCurr * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwmCurr * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwmCurr * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pwmCurr * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, pwmCurr * ui32Load / 1000);
}

void turnOffPwm() {
  ROM_PWMOutputInvert(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
  ROM_PWMOutputInvert(
      PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT,
      true);

  ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, false);
  ROM_PWMOutputState(
      PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT,
      false);
}

void updateGates() {
  int newABC = 0;
  a = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) & GPIO_PIN_4;
  b = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3) & GPIO_PIN_3;
  c = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) & GPIO_PIN_2;
  newABC = setAbc(a, b, c);
  if(hallIsCounting){
    if(newABC != abc) hallCount++;
  } else {
    hallCount = 0;
    hallIsCounting = true;
  }

  abc = newABC;
      // abc = 6;
      // switch1 = true;
      // isWithinCurrentBound = true;

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
  pwmCurr = pwmMIN;

  ROM_FPULazyStackingEnable();
  ROM_FPUEnable();

  SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                 SYSCTL_OSC_MAIN);
  ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

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
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);  // Motor enable

  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
  // Configure PWM
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);  // AH
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);  // AL
  ROM_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);  // BH
  ROM_GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);  // BL
  ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);  // CH
  ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);  // CL

  ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);
  ROM_GPIOPinConfigure(GPIO_PF3_M1PWM7);
  ROM_GPIOPinConfigure(GPIO_PC4_M0PWM6);
  ROM_GPIOPinConfigure(GPIO_PC5_M0PWM7);
  ROM_GPIOPinConfigure(GPIO_PE4_M0PWM4);
  ROM_GPIOPinConfigure(GPIO_PE5_M0PWM5);

  ui32PWMClock = SysCtlClockGet();
  ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

  PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);

  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load);

  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6 | PWM_OUT_7,
                       pwmMIN * ui32Load / 1000);
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5 | PWM_OUT_4 | PWM_OUT_6 | PWM_OUT_7,
                       pwmMIN * ui32Load / 1000);

  ROM_PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
  ROM_PWMOutputState(
      PWM0_BASE, PWM_OUT_5_BIT | PWM_OUT_4_BIT| PWM_OUT_6_BIT | PWM_OUT_7_BIT,
      true);

  ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);

  // Configure ADC
  ADCHardwareOversampleConfigure(ADC0_BASE, 4);
  ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

  // 1 6 5

  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH1);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH6);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH5);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH4);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH3);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH7);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 0);

  // GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);  // Current Sensor
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);  // Current Sensor 1
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);  // Current Sensor 2 
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);  // Current Sensor 3

  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);  // Throttle

  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);  // Boost Current
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);  // Boost Voltage
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);  // Battery Voltage

  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  ui32Period = (SysCtlClockGet()  / TIMER_FREQUENCY) - 1;
  TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period);
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
     isWithinCurrentBound = true;
     GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
   } else {
    isWithinCurrentBound = true;
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
   }
  updateGates();
}

void updatePWM(float dutyCycle){
  pwmCurr = dutyCycle * 1000;
  if (pwmCurr < pwmMIN) {
    pwmCurr = pwmMIN;
  } else if (pwmCurr > pwmMAX) {
    pwmCurr = pwmMAX;
  }
  setPulseWidth();
}

float getCurrentCommand(uint32_t digitalValue){
    if(digitalValue < thrMIN) digitalValue = thrMIN;\
    if(digitalValue > thrMAX) digitalValue = thrMAX;
  return (digitalValue - thrMIN)*1.0/ (thrMAX - thrMIN) * cur_cmd_MAX;
}

float getSpeedCommand(uint32_t digitalValue){
    if(digitalValue < thrMIN) digitalValue = thrMIN;\
    if(digitalValue > thrMAX) digitalValue = thrMAX;
  return (digitalValue - thrMIN)*1.0/ (thrMAX - thrMIN) * speed_cmd_MAX;
}

float getDutyCycleCommand(uint32_t digitalValue){
    if(digitalValue < thrMIN) digitalValue = thrMIN;\
    if(digitalValue > thrMAX) digitalValue = thrMAX;
  return (digitalValue - thrMIN)*1.0/ (thrMAX - thrMIN);
}

float getSensedCurrentFloat(uint32_t digitalValue){
  return  (digitalValue - is_Offset)/is_Slope;
}


uint32_t getMax(uint32_t value1,uint32_t value2,uint32_t value3){
  uint32_t temp;
  if(value1 > value2){
    temp = value1;
  } else {
    temp = value2;
  }

  if(value3 > temp) {
    temp = value3;
  }

  return temp;
}

void ADC0IntHandler(void) {
  uint32_t total = 0;
  float speedTotal = 0;

  uint8_t i = 0;
  ADCIntClear(ADC0_BASE, 0);
  ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value);
  throttle = ui32ADC0Value[3];

  battery_current = ui32ADC0Value[4];
  battery_voltage = ui32ADC0Value[6];
  boost_voltage = ui32ADC0Value[5];

  current_10[currentIndex] = getMax(ui32ADC0Value[0],ui32ADC0Value[1],ui32ADC0Value[2]);

  for(i = 0; i<10;i++){
    total += current_10[i];
  }
  sensedCurrent = total/10;

  currentIndex++;
  if(currentIndex > 9) {
    currentIndex = 0;
  }

  if(speedSensorCount++ > SPEED_SENSOR_DELAY) {
    speedSensorCount = 0;    
    sensedSpeed = 2.0*PI*hallCount*RADIUS*TIMER_FREQUENCY/(1.0*TICK_PER_REV*SPEED_SENSOR_DELAY);
    maxCount = hallCount;
    hallIsCounting = false;
  }

  sensedCurrentFloat = getSensedCurrentFloat(sensedCurrent);
  // dutyCycle = getDutyCycleCommand(throttle);
  // dutyCycle = sat_dual(dutyCycle,0.95,0.05);

  speedCommand = getSpeedCommand(throttle);
  // currentCommand = getCurrentCommand(throttle);

  currentCommand = pidloop(speedCommand, sensedSpeed, false,
   p_i_i_pos, p_i_i_neg, 0.0, cur_cmd_MAX, &s_int, &s_err);
  dutyCycle = pidloop(currentCommand, sensedCurrentFloat, !switch1, 
  p_i_d_pos, p_i_d_neg, 0.05, 0.95, &id_int, &id_err);
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

  if(switch1) {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);
  } else {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);
  }
  updateGates();
}

void Timer0IntHandler(void) { TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); }
