#ifndef __ANGLE_CONTROL_H
#define __ANGLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#define CAR_ANGLE_SET   2.2 //单位：度
#define CAR_ANGLE_SPEED_SET  0
/*直立控制周期*/
#define CONTROL_PERIOD 5 //5ms
  
extern float g_fCarAngle;
extern float g_fGyroscopeAngleSpeed;
extern float g_fAngleControlOut;

  
void AngleControl(void);

#ifdef __cplusplus
}
#endif
  
#endif /* __ANGLE_CONTROL_H */

