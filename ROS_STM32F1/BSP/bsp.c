#include "bsp.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

void bsp_init(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  //TIM1_TIMER_Init(999, 71); //��ʱû�õ�
  systick_init();	
  /*����USB ���⴮��*/
  delay_ms(1000);
  USB_Connection_Config(DISABLE); //USB�ȶϿ�
	delay_ms(500);
	USB_Connection_Config(ENABLE);	//USB�ٴ�����
  Set_USBClock(); //����USBʱ��48MHz
  USB_Interrupts_Config();
  USB_Init();
  /*Ӳ����ʼ��*/
  //while(MPU_DMP_Init()); //�ȴ�IMU��ʼ�����
  led_init();
  //TIM1_TIMER_Init(999, 71); //��ʱû�õ�
  motorInit();
  EncoderInit();
  while(MPU_DMP_Init()); //�ȴ�IMU��ʼ�����
  TIM1_TIMER_Init(999, 71); //��ʱû�õ�
}