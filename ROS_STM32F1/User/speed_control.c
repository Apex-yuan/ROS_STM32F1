#include "speed_control.h"
#include "encoder.h"
#include "protocol.h"
#include "config.h"
#include "variable.h"


float g_fCarSpeed, g_fCarSpeedOld;
//float g_fCarPosition;
int g_nLeftMotorPulseSigma, g_nRightMotorPulseSigma;
float g_fSpeedControlOutOld, g_fSpeedControlOutNew;
float g_fSpeedControlIntegral;
float g_fSpeedControlOut;
uint16_t g_nSpeedControlPeriod;

int16_t g_nLeftMotorPulse, g_nRightMotorPulse;
int16_t g_nLeftMotorPulseLast, g_nRightMotorPulseLast; 


float SPEED_CONTROL_P = 300;//300;//350;
float SPEED_CONTROL_I = 0.1;//0.6;//25;


//每5ms执行一次累加20次
void GetMotorPulse(void)
{
  int16_t nLeftMotorPulse,nRightMotorPulse;
  
  nLeftMotorPulse = (int16_t)TIM_GetCounter(TIM3);
  nRightMotorPulse = -(int16_t)TIM_GetCounter(TIM4);
  TIM_SetCounter(TIM3, 0);
  TIM_SetCounter(TIM4, 0);
  
  //平衡车部分
  g_nLeftMotorPulseSigma += nLeftMotorPulse;
  g_nRightMotorPulseSigma += nRightMotorPulse;
  //ROS部分
  left_encoder_count += nLeftMotorPulse;
  right_encoder_count += nRightMotorPulse;
}

//每100m执行一次
void SpeedControl(void)
{
  float fDelta;
  float fP,fI;
  int16_t nLeftMotorPulseDelta, nRightMotorPulseDelta;

  //获取ros下发的pid参数
  SPEED_CONTROL_P = ros_speed_kp;
  SPEED_CONTROL_I = ros_speed_ki;

  //获取编码器的数值
  g_nLeftMotorPulse = (int16_t)TIM_GetCounter(TIM3);
  g_nRightMotorPulse = -(int16_t)TIM_GetCounter(TIM4);
  nLeftMotorPulseDelta =  g_nLeftMotorPulse - g_nLeftMotorPulseLast;
  nRightMotorPulseDelta = g_nRightMotorPulse - g_nRightMotorPulseLast;
  g_nLeftMotorPulseLast = g_nLeftMotorPulse;
  g_nRightMotorPulseLast = g_nRightMotorPulse;

  //计算当前速度（转/秒）
  // g_fCarSpeed = (g_nLeftMotorPulseSigma + g_nRightMotorPulseSigma) / 2;
  // g_nLeftMotorPulseSigma = g_nRightMotorPulseSigma = 0;
  g_fCarSpeed = (nLeftMotorPulseDelta + nRightMotorPulseDelta) * 0.5;
  g_fCarSpeed *= CAR_SPEED_CONSTANT; //速度单位转化为：转/秒
  
  //速度控制PI运算
  fDelta = CAR_SPEED_SET - g_fCarSpeed + g_fBTSpeedSet + goal_velocity[LINEAR] * MPS2NPS; //m/s转化为转每秒  /周长
  fP = fDelta * SPEED_CONTROL_P;
  fI = fDelta * SPEED_CONTROL_I;
  
  g_fSpeedControlIntegral += fI;
  
  g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral;

}

//速度控制平滑输出，弱化速度控制对直立的影响
void SpeedControlOutput(void)
{
  float fDelta;
  
  fDelta = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
  g_fSpeedControlOut = fDelta * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;
  
  //虚拟示波器显示
  g_fware[3] = g_fSpeedControlOut;
  g_fware[4] = g_fCarSpeed*100;
}




