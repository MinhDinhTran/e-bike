#include "pid.h"

float pidloop(float y_c, float y, bool reset,
              float kp, float ki,
              float lowerLimit, float upperLimit, float Ts,
              float *integrator, float *error_d1) {
  if (reset) { // inplement reset
    *integrator = 0;
    *error_d1 = 0;
  }
  float error = y_c - y; // calc the current error
  float oldIntegrator = *integrator;   // store integrator for anti-windup
  *integrator = *integrator + (Ts / 2.0) * (error + *error_d1);  // integrate
  float u_unsat = kp * error + ki * *integrator; // calc desired output
  float u = sat_dual(u_unsat, upperLimit, lowerLimit); // saturate output
  *error_d1 = error;  // update the error for next time through
  if(u != u_unsat){ // implement integrator anti−windup
      *integrator = oldIntegrator;
  }
  return u;
}

float sat_dual(float in, float upperLimit, float lowerLimit) { // Saturate based on limits
  if (in > upperLimit) {
    return upperLimit;
  } else if (in < lowerLimit) {
    return lowerLimit;
  } else {
    return in;
  }
}
