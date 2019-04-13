#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif



extern float g_fCarAngle;
extern float g_fGyroscopeAngleSpeed;
extern float g_fAngleControlOut;
//extern short g_nGyro[3], g_nAccel[3];
//extern float g_fPitch, g_fRoll, g_fYaw;

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET  0
  
void AngleControl(void);

#ifdef __cplusplus
}
#endif
  
#endif /* __ANGLE_CONTROL_H */

