#include "../BSP/encoder/encoder.h"
//#include "../BSP/delay/delay.h"

void TIM3_ENCODER_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 使能定时器3的外设时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIO外设时钟使能                               	

   //设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //复用推挽输出
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseInitStructure.TIM_Period = 0xffff; //(ENCODER_RESOLUTION * 4) - 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x06;
  TIM_ICInit(TIM3,&TIM_ICInitStructure);
  
  TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
  
  //TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_SetCounter(TIM3,0);
  
  TIM_Cmd(TIM3, ENABLE);  //使能TIM1
}

void TIM4_ENCODER_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 使能定时器4的外设时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIO外设时钟使能                               	

   //设置该引脚为复用输出功能,输出TIM1 CH1的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //复用推挽输出
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 0xffff; //(ENCODER_RESOLUTION * 4) - 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 80K
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x06;
  TIM_ICInit(TIM4,&TIM_ICInitStructure);
  
  TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
  
  //TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_SetCounter(TIM4,0);
  
  TIM_Cmd(TIM4, ENABLE);  //使能TIM1
}

void EncoderInit(void)
{
  TIM3_ENCODER_Init();
  TIM4_ENCODER_Init();
}



//void encoder_setCounter(uint16_t counter)
//{
//  TIM_SetCounter(TIM2,counter);
//}

//int16_t encoder_getCounter(void)
//{
//  TIM_SetCounter(TIM2,0);
//  delay_ms(10);
//  return (int16_t)TIM_GetCounter(TIM2);   //这里利用了原码补码的特性可以输出正负
//  //printf("counter:%d\n",(int16_t)TIM_GetCounter(TIM2));
//}



