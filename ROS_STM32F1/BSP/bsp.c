#include "bsp.h"

void bsp_init(void)
{
  systick_init();	//滴答定时器初始化
  led_init();
  TIM1_TIMER_Init(999, 71);
  motorInit();
  EncoderInit();
  while(MPU_DMP_Init()); //等待IMU初始化完成
}