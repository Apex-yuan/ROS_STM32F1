#include "direction_control.h"
#include "config.h"

#include "mpu6050.h"

float g_fCardirection;

float g_fDirectionControlOut = 0;
float g_fDirectionControlNew, g_fDirectionControlOld;
float g_fDirectionControlIntegral;
uint8_t g_nDirectionControlPeriod;
uint16_t g_nDirectionControlCount;

float g_fBTDirectionSet;

extern float goal_velocity[WHEEL_NUM];
extern IMU_Data imu_data;
extern float ros_direction_kp;
extern float ros_direction_ki;

float DIRECTION_P = 0.5;
float DIRECTION_I = 0.5;


void DirectionControl()
{
 float theta, delta_theta;
 static float last_theta;
 float fDelta; 
 float fP,fI;

DIRECTION_P = ros_direction_kp;
DIRECTION_I = ros_direction_ki;
  //RAD2DEG
 theta = imu_data.rpy[2];
 delta_theta = theta -  last_theta;
 g_fCardirection = delta_theta /(1000.0 / DIRECTION_CONTROL_PERIOD);  //弧度/秒
 
 fDelta = DIRECTION_SET - imu_data.gyro[2] + goal_velocity[ANGULAR] + g_fBTDirectionSet;
  //g_fDirectionControlNew = goal_velocity[ANGULAR] * 200 + g_fBTDirectionSet; //同时支持ROS下发和蓝牙控制
  fP = fDelta * DIRECTION_P;
  fI = fDelta * DIRECTION_I;
  
  g_fDirectionControlIntegral += fI;
  g_fDirectionControlOld = g_fDirectionControlNew;
  g_fDirectionControlNew = fP + g_fDirectionControlIntegral;

  // fDelta = CAR_SPEED_SET - g_fCarSpeed + g_fBTSpeedSet + goal_velocity[LINEAR] * MPS2NPS; //m/s转化为转每秒  /周长
  // fP = fDelta * SPEED_CONTROL_P;
  // fI = fDelta * SPEED_CONTROL_I;
  
  // g_fSpeedControlIntegral += fI;
  
  // g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  // g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral;
}


void DirectionControlOutput(void)
{
  float fValue;
  
  fValue = g_fDirectionControlNew - g_fDirectionControlOld;
  g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) / 
                           DIRECTION_CONTROL_PERIOD + g_fDirectionControlOld; 
}


