#include "bsp.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

void bsp_init(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  systick_init();
  usart3_init(9600);
  led_init();	
  /*����USB ���⴮��*/
  delay_ms(1000);
  USB_Connection_Config(DISABLE); //USB�ȶϿ�
	delay_ms(500);
	USB_Connection_Config(ENABLE);	//USB�ٴ�����
  Set_USBClock(); //����USBʱ��48MHz
  USB_Interrupts_Config();
  USB_Init();
  /*Ӳ����ʼ��*/
  delay_ms(3000);
  while(MPU_DMP_Init()); //�ȴ�IMU��ʼ�����
  TIM1_TIMER_Init(999, 71); //��Ϊֱ���ڳ�ʼ����ʱ����˶�ʱ�������Ըú���Ӧ����while(MPU_DMP_Init());֮�󣬷�ֹ�ڳ�ʼ��mpu6050�Ĺ����д������жϽ��̣����³����ܷɡ�
  motor_init();
  encoder_init();
  //while(MPU_DMP_Init()); //�ȴ�IMU��ʼ�����
  //TIM1_TIMER_Init(999, 71); //
  led_on(LED0); //��ʼ����ɵ���LED��ָʾ��ʼ���ɹ�
  //GPIO_ResetBits(GPIOC,GPIO_Pin_13);
}