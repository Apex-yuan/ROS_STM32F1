#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#define CAR_ANGLE_SET   0
#define CAR_ANGLE_SPEED_SET  0

extern float g_fCarAngle;
extern float g_fGyroscopeAngleSpeed;
extern float g_fAngleControlOut;

  
void AngleControl(void);

#ifdef __cplusplus
}
#endif
  
#endif /* __ANGLE_CONTROL_H */

