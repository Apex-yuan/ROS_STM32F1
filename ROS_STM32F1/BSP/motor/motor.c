#include "../BSP/motor/motor.h"

// left motor:  gpio:PB13,PB12   PWM: PA3
// right motor: gpio:PB14,PB15   PWM: PA2
 static void TIM2_PWM_Init(uint16_t arr, uint16_t psc)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(MOTOR_TIM_CLK, ENABLE);
  RCC_APB2PeriphClockCmd(MOTOR_PWM_GPIO_CLK, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = L_MOTOR_PWM_PIN | R_MOTOR_PWM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MOTOR_PWM_GPIO_PORT, &GPIO_InitStructure);
  
  TIM_TimeBaseInitStructure.TIM_Period = arr;  
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc; 
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);

  //TIM_CtrlPWMOutputs(TIM2,ENABLE);	//MOE 主输出使能	

  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // 使能TIM3重载寄存器ARR
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2
    
}

void motor_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  TIM2_PWM_Init(999, 5);  //12KHz
  
  RCC_APB2PeriphClockCmd(MOTOR_IN_GPIO_CLK, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = L_MOTOR_IN1_PIN | L_MOTOR_IN2_PIN | R_MOTOR_IN1_PIN | R_MOTOR_IN2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MOTOR_IN_GPIO_PORT, &GPIO_InitStructure);
  GPIO_ResetBits(MOTOR_IN_GPIO_PORT, L_MOTOR_IN1_PIN | L_MOTOR_IN2_PIN | R_MOTOR_IN1_PIN | R_MOTOR_IN2_PIN); //电机默认设置为停止状态
}

void motor_setPwm(MotorChoice choice, uint16_t pwm)
{
  if(choice == LEFT_MOTOR)
  {
    TIM_SetCompare4(MOTOR_TIM, pwm);
  }
  if(choice == RIGHT_MOTOR)
  {
    TIM_SetCompare3(MOTOR_TIM, pwm);
  }
}

void motor_setDirection(MotorChoice choice, MotorDirection direction)
{
  if(choice == LEFT_MOTOR)
  {
    if(direction == FRONT)
    {
      GPIO_SetBits(MOTOR_IN_GPIO_PORT, L_MOTOR_IN1_PIN);
      GPIO_ResetBits(MOTOR_IN_GPIO_PORT, L_MOTOR_IN2_PIN);
    }
    else if(direction == BACK)
    {
      GPIO_SetBits(MOTOR_IN_GPIO_PORT, L_MOTOR_IN2_PIN);
      GPIO_ResetBits(MOTOR_IN_GPIO_PORT, L_MOTOR_IN1_PIN);
    }
    else if(direction == STOP)
    {
      GPIO_ResetBits(MOTOR_IN_GPIO_PORT, L_MOTOR_IN1_PIN | L_MOTOR_IN2_PIN);
    }
  }
  if(choice == RIGHT_MOTOR)
  {
    if(direction == FRONT)
    {
      GPIO_SetBits(MOTOR_IN_GPIO_PORT, R_MOTOR_IN1_PIN);
      GPIO_ResetBits(MOTOR_IN_GPIO_PORT, R_MOTOR_IN2_PIN);
    }
    else if(direction == BACK)
    {
      GPIO_SetBits(MOTOR_IN_GPIO_PORT, R_MOTOR_IN2_PIN);
      GPIO_ResetBits(MOTOR_IN_GPIO_PORT, R_MOTOR_IN1_PIN);
    }
    else if(direction == STOP)
    {
      GPIO_ResetBits(MOTOR_IN_GPIO_PORT, R_MOTOR_IN1_PIN | R_MOTOR_IN2_PIN);
    }
  }
}

