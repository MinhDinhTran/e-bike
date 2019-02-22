#ifndef __PROJECT_H__
#define __PROJECT_H__

int setAbc(bool a, bool b, bool c);
void pins();
void pwmControl(uint32_t gateH,uint32_t  gateH_other,uint32_t gateL, uint32_t gateOff1,uint32_t gateOff2,uint32_t gateOff3);
void ADC0IntHandler(void);
void GPIOIntHandler(void);
void GPIOBIntHandler(void);
void Timer0IntHandler(void);
void initTimer();
int main(void);
void setPulseWidth();

#endif  // __PROJECT_H__
