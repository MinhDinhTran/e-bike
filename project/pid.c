#include "pid.h"

float pidloop(float y_c, float y, bool reset,
              float p_i_pos, float p_i_neg,
              float lowerLimit, float upperLimit,
              float *u_d1, float *error_d1) {
  if (reset) { // inplement reset
    *error_d1 = 0;
    *u_d1 = 0;
  }
  float error = y_c - y; // calc the current error
  float u_unsat = *u_d1 + p_i_pos * error + p_i_neg * *error_d1; // calc desired output
  float u = sat_dual(u_unsat, upperLimit, lowerLimit); // saturate output
  *error_d1 = error;  // update the error for next time through
  *u_d1 = u;
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
