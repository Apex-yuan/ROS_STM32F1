#include "led.h"

void led_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(LED0_GPIO_CLK, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = LED0_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED0_GPIO_PORT, &GPIO_InitStructure);
  //½«LED0ÖÃÓÚ¹Ø±Õ×´Ì¬
  GPIO_SetBits(LED0_GPIO_PORT, LED0_GPIO_PIN);
}

void led_on(uint8_t LEDn)
{
  if((LEDn & LED0) == LED0)
  {
    GPIO_ResetBits(LED0_GPIO_PORT, LED0_GPIO_PIN);
  }
  if((LEDn & LED_ALL) == LED_ALL)
  {
    GPIO_ResetBits(LED0_GPIO_PORT, LED0_GPIO_PIN);
  }
}

void led_off(uint8_t LEDn)
{
  if((LEDn & LED0) == LED0)
  {
    GPIO_SetBits(LED0_GPIO_PORT, LED0_GPIO_PIN);
  }
  if((LEDn & LED_ALL) == LED_ALL)
  {
    GPIO_SetBits(LED0_GPIO_PORT, LED0_GPIO_PIN);
  }
}

void led_toggle(uint8_t LEDn)
{
  if((LEDn & LED0) == LED0)
  {
    GPIO_WriteBit(LED0_GPIO_PORT, LED0_GPIO_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED0_GPIO_PORT, LED0_GPIO_PIN)));
  }
  if((LEDn & LED_ALL) == LED_ALL)
  {
    GPIO_WriteBit(LED0_GPIO_PORT, LED0_GPIO_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED0_GPIO_PORT, LED0_GPIO_PIN)));
  }
  
}


