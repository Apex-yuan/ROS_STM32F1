#include "tim.h"

//#include "angle_control.h"
//#include "speed_control.h"
//#include "motor_control.h"

void TIM1_TIMER_Init(uint16_t arr, uint16_t psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  TIM_TimeBaseInitStructure.TIM_Period = arr;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
  
  //TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  TIM_ITConfig(  //使能或者失能指定的TIM1中断
		TIM1, //TIM1
		TIM_IT_Update  |  //TIM1 中断源
		TIM_IT_Trigger,   //TIM1 触发中断源 
		ENABLE  //使能
		);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_Cmd(TIM1, ENABLE);
}

////中断服务函数
//void TIM1_UP_IRQHandler(void) 
//{
//  if(TIM_GetFlagStatus(TIM1, TIM_IT_Update) != RESET)
//  {  
//    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//    //中断服务程序：
//    
//  }
//}

