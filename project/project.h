#ifndef __PROJECT_H__
#define __PROJECT_H__

int setAbc(bool a, bool b, bool c);
void updateGates(bool AH, bool AL, bool BH, bool BL, bool CH, bool CL);
void pins();
void ADC0IntHandler(void);
void GPIOIntHandler(void);
void GPIOBIntHandler(void);
void Timer0IntHandler(void);
void initTimer();
int main(void);

#endif  // __PROJECT_H__
