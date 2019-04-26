#include "speed_control.h"
#include "encoder.h"
#include "protocol.h"
#include "config.h"


float g_fCarSpeed,g_fCarSpeedOld;
float g_fCarPosition;
int g_nLeftMotorPulseSigma, g_nRightMotorPulseSigma;
float g_fSpeedControlOutOld, g_fSpeedControlOutNew;
float g_fSpeedControlIntegral;
float g_fSpeedControlOut;
uint16_t g_nSpeedControlPeriod;

float g_fBTSpeedSet = 0;

//��ʱΪros�������ݶ���ı������������Ż�
int left_encoder_count, right_encoder_count;

float SPEED_CONTROL_P = 300;//300;//350;
float SPEED_CONTROL_I = 0.1;//0.6;//25;

//������main.cpp�ļ��еı���
extern float goal_velocity[WHEEL_NUM];
extern float ros_speed_kp;
extern float ros_speed_ki;


//ÿ5msִ��һ���ۼ�20��
void GetMotorPulse(void)
{
  int16_t nLeftMotorPulse,nRightMotorPulse;
  
  nLeftMotorPulse = (int16_t)TIM_GetCounter(TIM3);
  nRightMotorPulse = -(int16_t)TIM_GetCounter(TIM4);
  TIM_SetCounter(TIM3,0);
  TIM_SetCounter(TIM4,0);
  
  //ƽ�⳵����
  g_nLeftMotorPulseSigma += nLeftMotorPulse;
  g_nRightMotorPulseSigma += nRightMotorPulse;
  //ROS����
  left_encoder_count += nLeftMotorPulse;
  right_encoder_count += nRightMotorPulse;
}


void SpeedControl(void)
{
  float fDelta;
  float fP,fI;
  
  //��ȡ�·���pid����
  SPEED_CONTROL_P = ros_speed_kp;
  SPEED_CONTROL_I = ros_speed_ki;

  g_fCarSpeed = (g_nLeftMotorPulseSigma + g_nRightMotorPulseSigma) / 2;
  g_nLeftMotorPulseSigma = g_nRightMotorPulseSigma = 0;
  g_fCarSpeed *= CAR_SPEED_CONSTANT; //�ٶȵ�λת��Ϊ��ת/��
  
  fDelta = CAR_SPEED_SET - g_fCarSpeed + g_fBTSpeedSet + goal_velocity[LINEAR] * MPS2NPS; //m/sת��Ϊתÿ��  /�ܳ�
  fP = fDelta * SPEED_CONTROL_P;
  fI = fDelta * SPEED_CONTROL_I;
  
  g_fSpeedControlIntegral += fI;
  
  g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral;

}

void SpeedControlOutput(void)
{
  float fDelta;
  
  fDelta = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
  g_fSpeedControlOut = fDelta * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;
  
  //����ʾ����
  g_fware[3] = g_fSpeedControlOut;
  g_fware[4] = g_fCarSpeed*100;
}




