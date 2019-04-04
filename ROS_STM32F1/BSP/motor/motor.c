#include "../BSP/motor/motor.h"

// left motor:  gpio:PB13,PB12   PWM: PA2
// right motor: gpio:PB14,PB15   PWM: PA3
 void TIM2_PWM_Init(uint16_t arr, uint16_t psc)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
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

  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);			 // 使能TIM3重载寄存器ARR
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2
    
}

void MotorInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  TIM2_PWM_Init(999, 5);  //12KHz
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
  
  
}


