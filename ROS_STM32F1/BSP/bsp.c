#include "bsp.h"

void bsp_init(void)
{
  systick_init();	//�δ�ʱ����ʼ��
  led_init();
  TIM1_TIMER_Init(999, 71); //��ʱû�õ�
  motorInit();
  EncoderInit();
  while(MPU_DMP_Init()); //�ȴ�IMU��ʼ�����
}