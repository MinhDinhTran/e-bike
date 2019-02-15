#ifndef __PID_H__
#define __PID_H__

float pidloop(float y_c, float y, bool reset, float kp, float ki, float kd,
              float limit, float Ts, float tau, float *integrator,
              float *error_d1, float *differentiator);

float sat(float in, float limit);

float sat(float in, float upperLimit, float lowerLimit);

#endif  // __PID_H__
