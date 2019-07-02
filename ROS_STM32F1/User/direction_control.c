#include "direction_control.h"
#include "config.h"
#include "variable.h"

float g_fCardirection; //角速度
float g_fDirectionControlOut = 0;
float g_fDirectionControlNew, g_fDirectionControlOld;
float g_fDirectionControlIntegral;
uint8_t g_nDirectionControlPeriod;
uint16_t g_nDirectionControlCount;

float DIRECTION_P = 0.5;
float DIRECTION_I = 0.5;


//每10ms执行一次
void DirectionControl(void)
{
  float theta, delta_theta;
  static float last_theta;
  float fDelta; 
  float fP,fI;

  //获取ros下发的方向PI参数
  DIRECTION_P = ros_direction_kp;
  DIRECTION_I = ros_direction_ki;
  
  //通过imu角度计算当前方向（角速度）
  theta = imu_data.rpy[2];
  delta_theta = theta -  last_theta;
  g_fCardirection = delta_theta /(1000.0 / DIRECTION_CONTROL_PERIOD);  //弧度/秒
 
  //通过odom计算当前方向（角速度）
  

  //方向控制PID运算
  fDelta = DIRECTION_SET - imu_data.gyro[2] + goal_velocity[ANGULAR] + g_fBTDirectionSet; //同时支持ROS下发和蓝牙控制
  fP = fDelta * DIRECTION_P;
  fI = fDelta * DIRECTION_I;
  
  g_fDirectionControlIntegral += fI;
  g_fDirectionControlOld = g_fDirectionControlNew;
  g_fDirectionControlNew = fP + g_fDirectionControlIntegral;
}

//方向控制平滑输出，弱化方向控制对直立控制的影响
void DirectionControlOutput(void)
{
  float fValue;
  
  fValue = g_fDirectionControlNew - g_fDirectionControlOld;
  g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) / DIRECTION_CONTROL_PERIOD + g_fDirectionControlOld; 
  
  //虚拟示波器显示
  g_fware[5] = g_fDirectionControlOut;
  g_fware[6] = g_fCardirection*100;
}


