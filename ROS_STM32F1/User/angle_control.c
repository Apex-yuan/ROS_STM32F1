#include "angle_control.h"
#include "mpu6050.h"
#include "config.h"
#include "variable.h"

/*角度控制，每5ms执行一次*/

float g_fCarAngle; //单位：rad
float g_fGyroscopeAngleSpeed; //单位：rad/s
float g_fAngleControlOut;

float ANGLE_CONTROL_P = 120; //64;
float ANGLE_CONTROL_D = 0; //120;//0.12;


void AngleControl(void)
{
  float fValue;

  //获取ros下发的角度pd参数
  ANGLE_CONTROL_P = ros_angle_kp;
  ANGLE_CONTROL_D = ros_angle_kd;

  //计算倾角（度）和角速度（度/秒）
	g_fCarAngle = RAD2DEG(imu_data.rpy[0]);
  g_fGyroscopeAngleSpeed = RAD2DEG(imu_data.gyro[0]);
  
  //角度控制PD计算
	fValue = (CAR_ANGLE_SET - g_fCarAngle) * ANGLE_CONTROL_P + 
           (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * ANGLE_CONTROL_D;
	g_fAngleControlOut = fValue;
  
  //跌倒检测
  if(g_fCarAngle > 50 || g_fCarAngle < (-50))
  {
    g_bFallFlag = 1;
  }
  else
  {
    g_bFallFlag = 0;
  }
  
  //虚拟示波器显示
//  g_fware[0] = g_fCarAngle;
//  g_fware[1] = g_fGyroscopeAngleSpeed;
//  g_fware[2] = g_fAngleControlOut;
}




