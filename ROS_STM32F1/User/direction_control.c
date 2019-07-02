#include "direction_control.h"
#include "config.h"
#include "variable.h"

float g_fCardirection; //���ٶ�
float g_fDirectionControlOut = 0;
float g_fDirectionControlNew, g_fDirectionControlOld;
float g_fDirectionControlIntegral;
uint8_t g_nDirectionControlPeriod;
uint16_t g_nDirectionControlCount;

float DIRECTION_P = 0.5;
float DIRECTION_I = 0.5;


//ÿ10msִ��һ��
void DirectionControl(void)
{
  float theta, delta_theta;
  static float last_theta;
  float fDelta; 
  float fP,fI;

  //��ȡros�·��ķ���PI����
  DIRECTION_P = ros_direction_kp;
  DIRECTION_I = ros_direction_ki;
  
  //ͨ��imu�Ƕȼ��㵱ǰ���򣨽��ٶȣ�
  theta = imu_data.rpy[2];
  delta_theta = theta -  last_theta;
  g_fCardirection = delta_theta /(1000.0 / DIRECTION_CONTROL_PERIOD);  //����/��
 
  //ͨ��odom���㵱ǰ���򣨽��ٶȣ�
  

  //�������PID����
  fDelta = DIRECTION_SET - imu_data.gyro[2] + goal_velocity[ANGULAR] + g_fBTDirectionSet; //ͬʱ֧��ROS�·�����������
  fP = fDelta * DIRECTION_P;
  fI = fDelta * DIRECTION_I;
  
  g_fDirectionControlIntegral += fI;
  g_fDirectionControlOld = g_fDirectionControlNew;
  g_fDirectionControlNew = fP + g_fDirectionControlIntegral;
}

//�������ƽ�����������������ƶ�ֱ�����Ƶ�Ӱ��
void DirectionControlOutput(void)
{
  float fValue;
  
  fValue = g_fDirectionControlNew - g_fDirectionControlOld;
  g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) / DIRECTION_CONTROL_PERIOD + g_fDirectionControlOld; 
  
  //����ʾ������ʾ
  g_fware[5] = g_fDirectionControlOut;
  g_fware[6] = g_fCardirection*100;
}


