/**
  ******************************************************************************
  * @file    usart1.c
  * @author  xiaoyuan
  * @version V2.0
  * @date    2018-1-28
  * @brief   USARTģ���ʼ���������ṩ�����ֲ�ͬ�ķ�ʽ֧��printf������
  *          �ṩ���жϽ��պ���
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "usart1.h"


/**
  * @brief  ��ʼ������1
  * @param  baud�����ڲ����ʣ�һ������Ϊ115200
  * @retval None
  */
void usart1_init(uint32_t bound) 
{
  /*�����ʼ���ṹ��*/
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	#if EN_USART1_RX		
	NVIC_InitTypeDef   NVIC_InitStructure;
	#endif
	
	/* ����GPIOA�˿�ʱ�Ӽ�USART1����ʱ�� */
	RCC_APB2PeriphClockCmd(USART1_CLK | USART1_GPIO_CLK |RCC_APB2Periph_AFIO, ENABLE);		//ʹ��USART1��GPIOA�Ͷ˿ڸ���ʱ��
 	
	/* ��λ����1 */
	USART_DeInit(USART1);  //��λ����1
	
	/* ����USART1_TX��PA9������GPIO״̬ */
  GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(USART1_TX_PORT, &GPIO_InitStructure);   

  /* ����USART1_RX��PA10������GPIO״̬ */
  GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  GPIO_Init(USART1_RX_PORT, &GPIO_InitStructure);  

  /* USART ��ʼ������ */
	USART_InitStructure.USART_BaudRate = bound;  //�����ʣ�һ������Ϊ115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ���Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //ֹͣλ��һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //У��λ������żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������:��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//����ģʽ���շ�ģʽ
  USART_Init(USART1, &USART_InitStructure);  //����ָ���Ĳ�����ʼ������1

  #if EN_USART1_RX  
  /* ����Ƕ�������жϿ�����NVIC */ 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  //����USART1Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	 //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
  /* ʹ�ܴ��ڽ����ж� */ 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  #endif
  /* ����1ʹ�� */
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ��� 
}

/* ѡ��ͬ�ķ�ʽ֧��printf������Ҫ����stdio.hͷ�ļ� */
#ifdef USE_MICROLIB  //ʹ��microLib�ķ���֧��printf���� 

/**
  * @brief  ���¶���c�⺯��printf������1�����¶�����ʹ��printf������ӡ��Ϣ
  * @param  ch��Ҫ���͵��ֽ�����
  * @param  *f
  * @retval ch
  */
int fputc(int ch, FILE *f)
{
  /* ����һ���ֽ����ݵ����� */
  USART_SendData(USART1, (uint8_t) ch);
  
  /* �ȴ�������� */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		

  return (ch);
}

/**
  * @brief  ���¶���c�⺯��scanf������1�����¶�����ʹ��scanf��getchar����
  * @param  *f
  * @retval ���ڽ��յ�������
  */
int fgetc(FILE *f)
{
  /* �ȴ������������� */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

  return (int)USART_ReceiveData(USART1);
}

#else  //��ʹ��microlib�Ϳ���֧��printf����  

#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */   
void _sys_exit(int x) 
{ 
	x = x; 
}

/**
  * @brief  ���¶���c�⺯��printf������1�����¶�����ʹ��printf������ӡ��Ϣ
  * @param  ch��Ҫ���͵��ֽ�����
  * @param  *f
  * @retval ch
  */ 
int fputc(int ch, FILE *f)
{      
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
  
  USART_SendData(USART1,(uint8_t)ch);   

	return ch;
}

#endif  //USE_MICROLIB

#if EN_USART1_RX   //���ʹ���˽���

///**
//  * @brief  ���¶���c�⺯��printf������1�����¶�����ʹ��printf������ӡ��Ϣ
//  * @param  ch��Ҫ���͵��ֽ�����
//  * @param  *f
//  * @retval ch
//  */ 
//void USART1_IRQHandler(void)                	//����1�жϷ������
//{
//	uint8_t rec;
//	
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d //0x0a��β)
//	{
//		rec = USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������

//  } 
//} 

#endif

