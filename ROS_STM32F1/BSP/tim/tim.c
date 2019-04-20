#include "tim.h"

void TIM1_TIMER_Init(uint16_t arr, uint16_t psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  TIM_TimeBaseInitStructure.TIM_Period = arr;
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;  //没有该句会出现定时不准确的情况，具体件log.md文件
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
  
  //TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  //TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
//  TIM_ITConfig(  //使能或者失能指定的TIM1中断
//		TIM1, //TIM1
//		TIM_IT_Update,//  |  //TIM1 中断源
//		/*TIM_IT_Trigger,*/   //TIM1 触发中断源 
//		ENABLE  //使能
//		);
  
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //没有该句程序会莫名跑飞（到硬件上访中断）
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  
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

