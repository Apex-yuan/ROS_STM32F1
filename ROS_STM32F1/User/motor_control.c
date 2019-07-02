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
    g_fLeftMotorOut = g_fAngleControlOut - g_fSpeedControlOut;
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut;
  #elif 0 
    g_fLeftMotorOut = -g_fDirectionControlOut;
    g_fRightMotorOut = g_fDirectionControlOut;
  #else  //正常模式
    g_fLeftMotorOut = g_fAngleControlOut - g_fSpeedControlOut - g_fDirectionControlOut;
    g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fDirectionControlOut;
  #endif
  g_fware[7] = g_fAngleControlOut - g_fSpeedControlOut;
  //g_fware[6] = g_fAngleControlOut - g_fSpeedControlOut;
  
  /*电机驱动为H桥电路上下晶体管不能同时导通，增加电机转速方向更换过程中的输出清零操作，以保护电机驱动器 （测试可行后在添加）*/  
   if(g_fLeftMotorOut > 0 && g_fCarSpeed < 0 || g_fLeftMotorOut < 0 && g_fCarSpeed > 0)
   {
     motor_setPwm(LEFT_MOTOR, (uint16_t) 0);
   }
   if(g_fRightMotorOut > 0 && g_fCarSpeed < 0 || g_fRightMotorOut < 0 && g_fCarSpeed > 0)
   {
     motor_setPwm(RIGHT_MOTOR, (uint16_t) 0);
   }

  //左轮电机
  if(g_fLeftMotorOut > 0)
  {
    motor_setDirection(LEFT_MOTOR, FRONT);
    g_fLeftMotorOut = g_fLeftMotorOut + LEFT_MOTOR_OUT_DEAD_ZONE; // +
  }
  else
  {
    motor_setDirection(LEFT_MOTOR, BACK);
    g_fLeftMotorOut = g_fLeftMotorOut - LEFT_MOTOR_OUT_DEAD_ZONE; // +
  }
  //右轮电机
  if(g_fRightMotorOut > 0)
  {
    motor_setDirection(RIGHT_MOTOR, FRONT);
    g_fRightMotorOut = g_fRightMotorOut + RIGHT_MOTOR_OUT_DEAD_ZONE; // +
  }
  else
  {
    motor_setDirection(RIGHT_MOTOR, BACK);
    g_fRightMotorOut = g_fRightMotorOut - RIGHT_MOTOR_OUT_DEAD_ZONE; // +
  }

  //电机输出限幅
  g_fLeftMotorOut = constrain(g_fLeftMotorOut, MIN_MOTOR_OUT, MAX_MOTOR_OUT);
  g_fRightMotorOut = constrain(g_fRightMotorOut, MIN_MOTOR_OUT, MAX_MOTOR_OUT);

  motor_setPwm(LEFT_MOTOR, (uint16_t) fabs(g_fLeftMotorOut));
  motor_setPwm(RIGHT_MOTOR, (uint16_t) fabs(g_fRightMotorOut));
//  motor_setDirection(LEFT_MOTOR, FRONT);
//  motor_setDirection(RIGHT_MOTOR, BACK);
//  motor_setPwm(LEFT_MOTOR, 200);
//  motor_setPwm(RIGHT_MOTOR, 200);
  
  //跌倒关闭电机输出
  if(g_bFallFlag == 1)
	{
		motor_setPwm(LEFT_MOTOR, 0);
    motor_setPwm(RIGHT_MOTOR, 0); 
	}
}




