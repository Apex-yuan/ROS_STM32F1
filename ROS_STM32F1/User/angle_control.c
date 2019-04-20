#include "angle_control.h"
#include "mpu6050.h"
#include "config.h"
#include "protocol.h"

float g_fCarAngle; //单位：rad
float g_fGyroscopeAngleSpeed; //单位：rad/s
float g_fAngleControlOut;
//short g_nGyro[3], g_nAccel[3];
//float g_fPitch, g_fRoll, g_fYaw;

float ANGLE_CONTROL_P = 60; //64;
float ANGLE_CONTROL_D = 0.7; //120;//0.12;

extern IMU_Data imu_data;  //from main.c

void AngleControl(void)
{
  float fValue;
	g_fCarAngle = RAD2DEG(imu_data.rpy[1]);//imu_data.rpy[1];//g_fRoll;
  g_fGyroscopeAngleSpeed = RAD2DEG(imu_data.gyro[0]);//(float)g_nGyro[0];
  
	fValue = (g_fCarAngle - CAR_ANGLE_SET) * ANGLE_CONTROL_P + 
           (g_fGyroscopeAngleSpeed - CAR_ANGLE_SPEED_SET) * ANGLE_CONTROL_D;
	g_fAngleControlOut = fValue;
  
  //虚拟示波器
  g_fware[0] = g_fCarAngle;
  g_fware[1] = g_fGyroscopeAngleSpeed;
  g_fware[2] = g_fAngleControlOut;
}




