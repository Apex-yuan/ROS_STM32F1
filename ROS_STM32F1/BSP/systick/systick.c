#ifdef __cplusplus
extern "C" {
#endif

#include "systick.h"

volatile uint32_t _us_tick;
volatile uint32_t _ms_tick;

void systick_init(void) 
{
//  NVIC_InitTypeDef NVIC_InitStructure;
 
  _us_tick = 0;
  _ms_tick = 0;  
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
  
	SysTick_Config(SystemCoreClock / 1000000);
}

void delay_ms(uint32_t millis) 
{ 
	uint32_t target;
	
	target = _ms_tick + millis;
	while(_ms_tick < target);
} 
void delay_us(uint32_t uillis)
{ 
	uint32_t target;
	target = _us_tick + uillis;
	while(_us_tick < target);
}
void SysTick_Handler(void) 
{
	_us_tick++;
  _ms_tick = _us_tick / 1000;
}

uint32_t millis(void) 
{
	return _ms_tick;
}
uint32_t micros(void)
{
  return _us_tick;
}

void reset(void) 
{
	_us_tick = 0;
  _ms_tick = 0;
}

#ifdef __cplusplus
}
#endif

