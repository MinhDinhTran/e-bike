#include <stdbool.h>

#ifndef __PID_H__
#define __PID_H__

float pidloop(float y_c, float y, bool reset, float kp, float ki,
              float lowerLimit, float upperLimit, float *integrator,
              float *error_d1);

float sat_dual(float in, float upperLimit, float lowerLimit);

#endif  // __PID_H__
