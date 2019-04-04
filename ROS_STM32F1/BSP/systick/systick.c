#ifdef __cplusplus
extern "C" {
#endif

#include "systick.h"

volatile uint32_t _counter;

void systick_init(void) 
{
  NVIC_InitTypeDef NVIC_InitStructure;
 
  _counter = 0;  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	SysTick_Config(SystemCoreClock / 1000);
}

void delay_ms(uint32_t millis) 
{ 
	uint32_t target;
	
	target = _counter + millis;
	while(_counter < target);
} 
void delay_us(uint32_t uillis)
{ 
	uint32_t target;
  SysTick_Config(SystemCoreClock / 100000);
	target = _counter + uillis;
	while(_counter < target);
}
void SysTick_Handler(void) 
{
	_counter++;
}

uint32_t millis(void) 
{
	return _counter;
}

void reset(void) 
{
	_counter = 0;
}

#ifdef __cplusplus
}
#endif

