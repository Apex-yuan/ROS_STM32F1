#include "direction_control.h"
#include "config.h"

#include "mpu6050.h"

float g_fDirectionControlOut = 0;
float g_fDirectionControlNew, g_fDirectionControlOld;
uint8_t g_nDirectionControlPeriod;
uint16_t g_nDirectionControlCount;

float g_fBTDirectionSet;

extern float goal_velocity[WHEEL_NUM];
extern IMU_Data imu_data;

float DIRECTION_P = 0.5;
float DIRECTION_D = 0.5;


void DirectionControl()
{
//  float theta, delta_theta;
//  static float last_theta;
//  float fValue; 
//  float w;
  //RAD2DEG
//  theta = RAD2DEG(imu_data.rpy[2]);
//  delta_theta = theta -  last_theta;
//  w = delta_theta / DIRECTION_CONTROL_PERIOD;  //度/秒
//  
//  fValue = DIRECTION_SET - w + RAD2DEG(goal_velocity[ANGULAR]);
  g_fDirectionControlNew = goal_velocity[ANGULAR] * 200 + g_fBTDirectionSet; //同时支持ROS下发和蓝牙控制
  
  g_fDirectionControlOld = g_fDirectionControlNew;
}


void DirectionControlOutput(void)
{
  float fValue;
  
  fValue = g_fDirectionControlNew - g_fDirectionControlOld;
  g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) / 
                           DIRECTION_CONTROL_PERIOD + g_fDirectionControlOld; 
}


