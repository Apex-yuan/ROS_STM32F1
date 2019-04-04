#include "angle_control.h"
#include "mpu6050.h"
#include "protocol.h"

float g_fCarAngle;
float g_fGyroscopeAngleSpeed;
float g_fAngleControlOut;
short g_nGyro[3], g_nAccel[3];
float g_fPitch, g_fRoll, g_fYaw;

float ANGLE_CONTROL_P = 64;//64;//80;//90
float ANGLE_CONTROL_D = 0.12;//0.12;//0.15;//0.2

void AngleControl(void)
{
  float fValue;
	g_fCarAngle = g_fRoll;
  g_fGyroscopeAngleSpeed = (float)g_nGyro[0];
  
	fValue = (g_fCarAngle - CAR_ANGLE_SET) * ANGLE_CONTROL_P + 
           (g_fGyroscopeAngleSpeed - CAR_ANGLE_SPEED_SET) * ANGLE_CONTROL_D;
	g_fAngleControlOut = fValue;
  
  //ÐéÄâÊ¾²¨Æ÷
  g_fware[0] = g_fCarAngle;
  g_fware[1] = g_fGyroscopeAngleSpeed;
  g_fware[2] = g_fAngleControlOut;
}




