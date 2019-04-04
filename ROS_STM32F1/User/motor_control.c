#include "motor_control.h"
#include "angle_control.h"
#include "speed_control.h"
#include "direction_control.h"
#include "protocol.h"

float g_fLeftMotorOut = 0;
float g_fRightMotorOut = 0;


void MotorOutput(void)
{
  int nLeft,nRight;
  
  #if 0
    g_fLeftMotorOut = 0;
    g_fRightMotorOut = 0;
  #elif 0   
    g_fLeftMotorOut = g_fAngleControlOut;
    g_fRightMotorOut = g_fAngleControlOut;
  #elif 0
    g_fLeftMotorOut = g_fAngleControlOut + g_fSpeedControlOut;
    g_fRightMotorOut = g_fAngleControlOut + g_fSpeedControlOut;
  #else
    g_fLeftMotorOut = g_fAngleControlOut + g_fSpeedControlOut - g_fDirectionControlOut;
    g_fRightMotorOut = g_fAngleControlOut + g_fSpeedControlOut + g_fDirectionControlOut;
  #endif
  g_fware[5] = g_fAngleControlOut + g_fSpeedControlOut;
  g_fware[6] = g_fAngleControlOut + g_fSpeedControlOut;
  
  if(g_fLeftMotorOut >= 1000)
    g_fLeftMotorOut = 1000;
  if(g_fRightMotorOut >= 1000)
    g_fRightMotorOut = 1000;
  if(g_fLeftMotorOut <= -1000)
    g_fLeftMotorOut = -1000;
  if(g_fRightMotorOut <= -1000)
    g_fRightMotorOut = -1000;
  
  nLeft = (int)g_fLeftMotorOut;
  nRight = (int)g_fRightMotorOut;
  
  if(nLeft < 0)
  {
    //g_fLeftMotorOut += MOTOR_OUT_DEAD_VAL;
    GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
    GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
    nLeft = (-nLeft);
  }
  else
  {
    //g_fLeftMotorOut -= MOTOR_OUT_DEAD_VAL;
    GPIO_SetBits(GPIOB,GPIO_Pin_15);
    GPIO_ResetBits(GPIOB,GPIO_Pin_14);
    nLeft = nLeft;
  }
  
  if(nRight < 0)
  {
    //g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
    GPIO_SetBits(GPIOB,GPIO_Pin_13);
    GPIO_ResetBits(GPIOB,GPIO_Pin_12);
    nRight = (-nRight);
  }
  else
  {
    //g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;
    GPIO_SetBits(GPIOB,GPIO_Pin_12);
    GPIO_ResetBits(GPIOB,GPIO_Pin_13);
    nRight = nRight;
  }
  
  TIM_SetCompare3(TIM2,(uint16_t)nLeft);
  TIM_SetCompare4(TIM2,(uint16_t)nRight);
  
  if(g_fCarAngle > 50 || g_fCarAngle < (-50))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
	}
}




