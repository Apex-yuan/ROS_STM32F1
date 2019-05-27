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
  /*配置USB 虚拟串口*/
  delay_ms(1000);
  USB_Connection_Config(DISABLE); //USB先断开
	delay_ms(500);
	USB_Connection_Config(ENABLE);	//USB再次连接
  Set_USBClock(); //设置USB时钟48MHz
  USB_Interrupts_Config();
  USB_Init();
  /*硬件初始化*/
  delay_ms(3000);
  while(MPU_DMP_Init()); //等待IMU初始化完成
  TIM1_TIMER_Init(999, 71); //因为直接在初始化的时候打开了定时器，所以该函数应放在while(MPU_DMP_Init());之后，防止在初始化mpu6050的过程中触发了中断进程，导致程序跑飞。
  motor_init();
  encoder_init();
  //while(MPU_DMP_Init()); //等待IMU初始化完成
  //TIM1_TIMER_Init(999, 71); //
  led_on(LED0); //初始化完成点亮LED灯指示初始化成功
  //GPIO_ResetBits(GPIOC,GPIO_Pin_13);
}