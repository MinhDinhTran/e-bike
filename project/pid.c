#include <stdbool.h>
#include "pid.h"

float pidloop(float y_c, float y, bool reset,
              float kp, float ki, float kd,
              float lowerLimit, float upperLimit, float Ts,
              float *integrator, float *error_d1) {
  // reset(initialize) persistent variables when flag==1
  if (reset) {
    *integrator = 0;
    *error_d1 = 0;
  }
  // compute the current error
  float error = y_c - y;
  // update integrator
  *integrator = *integrator + (Ts / 2) * (error + *error_d1);
  *error_d1 = error;  // update the error for next time through
  float u_unsat = kp * error + ki * *integrator;
  float u = sat_dual(u_unsat, upperLimit, lowerLimit);
  // implement integrator anti−windup
  if (error > 0.5 || error < -0.5) {
     *integrator = *integrator + Ts / ki * (u - u_unsat);
  }
  return u;
}

float sat_dual(float in, float upperLimit, float lowerLimit) {
  if (in > upperLimit) {
    return upperLimit;
  } else if (in < lowerLimit) {
    return lowerLimit;
  } else {
    return in;
  }
}
