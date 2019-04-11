#include "bsp.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

void bsp_init(void)
{
  systick_init();	
  /*配置USB 虚拟串口*/
  USB_Connection_Config(DISABLE); //USB先断开
	delay_ms(500);
	USB_Connection_Config(ENABLE);	//USB再次连接
  Set_USBClock(); //设置USB时钟48MHz
  USB_Interrupts_Config();
  USB_Init();
  /*硬件初始化*/
  led_init();
  TIM1_TIMER_Init(999, 71); //暂时没用到
  motorInit();
  EncoderInit();
  //while(MPU_DMP_Init()); //等待IMU初始化完成
}