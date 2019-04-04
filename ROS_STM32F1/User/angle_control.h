#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#include "mpu6050.h"


extern float g_fCarAngle;
extern float g_fGyroscopeAngleSpeed;
extern float g_fAngleControlOut;
extern short g_nGyro[3], g_nAccel[3];
extern float g_fPitch, g_fRoll, g_fYaw;

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET  0

void AngleControl(void);




#endif /* __ANGLE_CONTROL_H */

