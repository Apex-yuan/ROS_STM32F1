#include "angle_control.h"
#include "mpu6050.h"
#include "config.h"
#include "variable.h"

/*�Ƕȿ��ƣ�ÿ5msִ��һ��*/

float g_fCarAngle; //��λ��rad
float g_fGyroscopeAngleSpeed; //��λ��rad/s
float g_fAngleControlOut;

float ANGLE_CONTROL_P = 120; //64;
float ANGLE_CONTROL_D = 0; //120;//0.12;


void AngleControl(void)
{
  float fValue;

  //��ȡros�·��ĽǶ�pd����
  ANGLE_CONTROL_P = ros_angle_kp;
  ANGLE_CONTROL_D = ros_angle_kd;

  //������ǣ��ȣ��ͽ��ٶȣ���/�룩
	g_fCarAngle = RAD2DEG(imu_data.rpy[0]);
  g_fGyroscopeAngleSpeed = RAD2DEG(imu_data.gyro[0]);
  
  //�Ƕȿ���PD����
	fValue = (CAR_ANGLE_SET - g_fCarAngle) * ANGLE_CONTROL_P + 
           (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * ANGLE_CONTROL_D;
	g_fAngleControlOut = fValue;
  
  //�������
  if(g_fCarAngle > 50 || g_fCarAngle < (-50))
  {
    g_bFallFlag = 1;
  }
  else
  {
    g_bFallFlag = 0;
  }
  
  //����ʾ������ʾ
//  g_fware[0] = g_fCarAngle;
//  g_fware[1] = g_fGyroscopeAngleSpeed;
//  g_fware[2] = g_fAngleControlOut;
}




