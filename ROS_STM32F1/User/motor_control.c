#include "motor_control.h"
#include "angle_control.h"
#include "speed_control.h"
#include "direction_control.h"
#include "motor.h"
#include <math.h>
#include "variable.h"
#include "config.h"

static float g_fLeftMotorOut = 0;
static float g_fRightMotorOut = 0;


void MotorOutput(void)
{ 
  #if 0  //关闭电机输出
    g_fLeftMotorOut = 0;
    g_fRightMotorOut = 0;
  #elif 0  //直立控制调试 
    g_fLeftMotorOut = g_fAngleControlOut;
    g_fRightMotorOut = g_fAngleControlOut;
  #elif 0 //直立+速度控制调试
    g_fLeftMotorOut = g_fAngleControlOut + g_fSpeedControlOut;
    g_fRightMotorOut = g_fAngleControlOut + g_fSpeedControlOut;
  #else  //正常模式
    g_fLeftMotorOut = g_fAngleControlOut - g_fSpeedControlOut - g_fDirectionControlOut;
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fDirectionControlOut;
  #endif
//  g_fware[5] = g_fAngleControlOut + g_fSpeedControlOut;
//  g_fware[6] = g_fAngleControlOut + g_fSpeedControlOut;
  
  //左轮电机
  if(g_fLeftMotorOut > 0)
  {
    setMotorDirection(LEFT_MOTOR, FRONT);
    g_fLeftMotorOut = g_fLeftMotorOut + LEFT_MOTOR_OUT_DEAD_ZONE; // +
  }
  else
  {
    setMotorDirection(LEFT_MOTOR, BACK);
    g_fLeftMotorOut = g_fLeftMotorOut - LEFT_MOTOR_OUT_DEAD_ZONE; // +
  }
  //右轮电机
  if(g_fRightMotorOut > 0)
  {
    setMotorDirection(RIGHT_MOTOR, FRONT);
    g_fRightMotorOut = g_fRightMotorOut + RIGHT_MOTOR_OUT_DEAD_ZONE; // +
  }
  else
  {
    setMotorDirection(RIGHT_MOTOR, BACK);
    g_fRightMotorOut = g_fRightMotorOut - RIGHT_MOTOR_OUT_DEAD_ZONE; // +
  }

  //电机输出限幅
  g_fLeftMotorOut = constrain(g_fLeftMotorOut, MIN_MOTOR_OUT, MAX_MOTOR_OUT);
  g_fRightMotorOut = constrain(g_fRightMotorOut, MIN_MOTOR_OUT, MAX_MOTOR_OUT);

  setMotorPwm(LEFT_MOTOR, (uint16_t) fabs(g_fLeftMotorOut));
  setMotorPwm(RIGHT_MOTOR, (uint16_t) fabs(g_fRightMotorOut));
//  setMotorDirection(LEFT_MOTOR, FRONT);
//  setMotorDirection(RIGHT_MOTOR, BACK);
//  setMotorPwm(LEFT_MOTOR, 200);
//  setMotorPwm(RIGHT_MOTOR, 200);
  
  //跌倒关闭电机输出
  if(g_fCarAngle > 50 || g_fCarAngle < (-50))
	{
		setMotorPwm(LEFT_MOTOR, 0);
    setMotorPwm(RIGHT_MOTOR, 0); 
	}
}




