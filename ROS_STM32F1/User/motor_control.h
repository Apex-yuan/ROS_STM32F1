#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor.h"

extern float g_fLeftMotorOut;
extern float g_fRightMotorOut;

void MotorOutput(void);

#ifdef __cplusplus
}
#endif

#endif

