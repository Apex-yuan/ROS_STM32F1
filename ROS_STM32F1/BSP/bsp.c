#include "bsp.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

void bsp_init(void)
{
  systick_init();	//�δ�ʱ����ʼ��
  
  delay_ms(1800);
  USB_Port_Set(0); 	//USB�ȶϿ�
	delay_ms(700);
	USB_Port_Set(1);	//USB�ٴ�����
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  
  
  led_init();
  TIM1_TIMER_Init(999, 71); //��ʱû�õ�
  motorInit();
  EncoderInit();
  //while(MPU_DMP_Init()); //�ȴ�IMU��ʼ�����
}