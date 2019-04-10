#include "bsp.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

void bsp_init(void)
{
  systick_init();	//滴答定时器初始化
  
  delay_ms(1800);
  USB_Port_Set(0); 	//USB先断开
	delay_ms(700);
	USB_Port_Set(1);	//USB再次连接
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  
  
  led_init();
  TIM1_TIMER_Init(999, 71); //暂时没用到
  motorInit();
  EncoderInit();
  //while(MPU_DMP_Init()); //等待IMU初始化完成
}