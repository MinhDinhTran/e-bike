#include <stdbool.h>

float pidloop(float y_c, float y, bool reset, float kp, float ki, float kd,
              float limit, float Ts, float tau, float *integrator,
              float *error_d1, float *differentiator) {
  // reset(initialize) persistent variables when flag==1
  if (reset) {
    *integrator = 0;
    *differentiator = 0;
    *error_d1 = 0;
  }
  // compute the current error
  float error = y_c - y;
  // update integrator
  *integrator = *integrator + (Ts / 2) * (error + *error_d1);
  // update differentiator
  *differentiator = (2 * tau - Ts) / (2 * tau + Ts) * *differentiator +
                    2 / (2 * tau + Ts) * (error - *error_d1);
  *error_d1 = error;  // update the error for next time through
  float u_unsat = kp * error + ki * *integrator + kd * *differentiator;
  float u = sat(u_unsat, limit);
  // implement integrator anti−windup
  if (ki != 0) {
    *integrator = *integrator + Ts / ki * (u - u_unsat);
  }
  return u;
}

float sat(float in, float limit) { return sat(in, limit, -limit); }

float sat(float in, float upperLimit, float lowerLimit) {
  if (in > upperLimit) {
    return upperLimit;
  } else if (in < lowerLimit) {
    return lowerLimit;
  } else {
    return in;
  }
}