#ifndef __MOTOR_OUTPUT_H
#define __MOTOR_OUTPUT_H

#include <math.h>

#ifdef __cplusplus
 extern "C" {
#endif

void motorControl(float linear_vel, float angular_vel);

#ifdef __cplusplus
 }
#endif

#endif /* __MOTOR_OUTPUT_H */
