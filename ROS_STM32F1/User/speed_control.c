#include "speed_control.h"
#include "encoder.h"
#include "protocol.h"


float g_fCarSpeed,g_fCarSpeedOld;
float g_fCarPosition;
int g_nLeftMotorPulseSigma, g_nRightMotorPulseSigma;
float g_fSpeedControlOutOld, g_fSpeedControlOutNew;
float g_fSpeedControlIntegral;
float g_fSpeedControlOut;
uint16_t g_nSpeedControlPeriod;

float g_fBTSpeedSet = 0;

float SPEED_CONTROL_P = 300;//300;//350;
float SPEED_CONTROL_I = 0.6;//25;


//每5ms执行一次累加20次
void GetMotorPulse(void)
{
  int32_t nLeftMotorPulse,nRightMotorPulse;
  
  nLeftMotorPulse = (int16_t)TIM_GetCounter(TIM3);
  nRightMotorPulse = -(int16_t)TIM_GetCounter(TIM4);
  TIM_SetCounter(TIM3,0);
  TIM_SetCounter(TIM4,0);
  
  g_nLeftMotorPulseSigma += nLeftMotorPulse;
  g_nRightMotorPulseSigma += nRightMotorPulse;
}


void SpeedControl(void)
{
  float fDelta;
  float fP,fI;
  
  g_fCarSpeed = (g_nLeftMotorPulseSigma + g_nRightMotorPulseSigma) / 2;
  g_nLeftMotorPulseSigma = g_nRightMotorPulseSigma = 0;
  g_fCarSpeed *= CAR_SPEED_CONSTANT; //速度单位转化为：转/秒
  
  fDelta = CAR_SPEED_SET - g_fCarSpeed + g_fBTSpeedSet;
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
  
  //虚拟示波器
  g_fware[3] = g_fSpeedControlOut;
  g_fware[4] = g_fCarSpeed*100;
}




