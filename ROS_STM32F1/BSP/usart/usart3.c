/**
  ******************************************************************************
  * @file    usart3.c
  * @author  xiaoyuan
  * @version V1.0
  * @date    2018-11-06
  * @brief   USART2ģ���ʼ��������
  *          �ṩ���жϽ��պ���
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "usart3.h"
#include <stdarg.h>
#include <string.h> 
#include <stdio.h>

//static char * itoa( int value, char *string, int radix );


RxFramTypeDef rxFram = {0};

/**
  * @brief  ��ʼ������3
  * @param  bound�����ڲ����ʣ�һ������Ϊ115200
  * @retval None
  */
void usart3_init(uint32_t bound) 
{
  /*�����ʼ���ṹ��*/
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* ����GPIOB�˿�ʱ�Ӽ�USART3����ʱ�� */
  RCC_APB1PeriphClockCmd(USART3_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(USART3_GPIO_CLK, ENABLE); 
 	
	/* ��λ����1 */
	USART_DeInit(USART3);  //��λ����1
	
	/* ����USART3_TX��PB10������GPIO״̬ */
  GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(USART3_TX_PORT, &GPIO_InitStructure);   

  /* ����USART3_RX��PB11������GPIO״̬ */
  GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(USART3_RX_PORT, &GPIO_InitStructure);  

  /* USART ��ʼ������ */
	USART_InitStructure.USART_BaudRate = bound;  //�����ʣ�һ������Ϊ115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //�ֳ���Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //ֹͣλ��һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;  //У��λ������żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//Ӳ��������:��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//����ģʽ���շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);  //����ָ���Ĳ�����ʼ������1
  
  /* ����Ƕ�������жϿ�����NVIC */ 
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  //����USART1Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	 //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	
  /* ʹ�ܴ��ڽ����ж� */ 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  /* ʹ�ܴ�������ж� */
  //USART_ITConfig (USART3, USART_IT_ORE, ENABLE );
  /* ����3ʹ�� */
  USART_Cmd(USART3, ENABLE);
}


//����2,printf ����
//ȷ��һ�η������ݲ�����USART2_MAX_SEND_LEN�ֽ�
void usart3_printf(char* fmt,...)  
{
  char buffer[USART3_BUFFER_LEN+1];  
	uint16_t i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)buffer,fmt,ap);
	va_end(ap);
	i=strlen((const char*)buffer);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
		USART_SendData(USART3,buffer[j]); 
	} 
}



///*
// * ��������usart3_printf
// * ����  ����ʽ�������������C���е�printf��������û���õ�C��
// * ����  ��-Data   Ҫ���͵����ڵ����ݵ�ָ��
// *			   -...    ��������
// * ���  ����
// * ����  ���� 
// * ����  ���ⲿ����
// *         ����Ӧ��USART2_printf("\r\n this is a demo \r\n" );
// *            		 USART2_printf("\r\n %d \r\n", i );
// *            		 USART2_printf("\r\n %s \r\n", j );
// */
//void usart3_printf (char * Data, ... )
//{
//	const char *s;
//	int d;   
//	char buf[16];

//	
//	va_list ap;
//	va_start(ap, Data);

//	while ( * Data != 0 )     // �ж��Ƿ񵽴��ַ���������
//	{				                          
//		if ( * Data == 0x5c )  //'\'
//		{									  
//			switch ( *++Data )
//			{
//				case 'r':							          //�س���
//				USART_SendData(USART3, 0x0d);
//				Data ++;
//				break;

//				case 'n':							          //���з�
//				USART_SendData(USART3, 0x0a);	
//				Data ++;
//				break;

//				default:
//				Data ++;
//				break;
//			}			 
//		}
//		
//		else if ( * Data == '%')
//		{									  //
//			switch ( *++Data )
//			{				
//				case 's':										  //�ַ���
//				s = va_arg(ap, const char *);
//				
//				for ( ; *s; s++) 
//				{
//					USART_SendData(USART3,*s);
//					while( USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET );
//				}
//				
//				Data++;
//				
//				break;

//				case 'd':			
//					//ʮ����
//				d = va_arg(ap, int);
//				
//				itoa(d, buf, 10);
//				
//				for (s = buf; *s; s++) 
//				{
//					USART_SendData(USART3,*s);
//					while( USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET );
//				}
//				
//				Data++;
//				
//				break;
//				
//				default:
//				Data++;
//				
//				break;
//				
//			}		 
//		}
//		
//		else USART_SendData(USART3, *Data++);
//		
//		while ( USART_GetFlagStatus ( USART3, USART_FLAG_TXE ) == RESET );
//		
//	}
//}


///*
// * ��������itoa
// * ����  ������������ת�����ַ���
// * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
// *         -value Ҫת����������
// *         -buf ת������ַ���
// *         -radix = 10
// * ���  ����
// * ����  ����
// * ����  ����USART2_printf()����
// */
//static char * itoa( int value, char *string, int radix )
//{
//	int     i, d;
//	int     flag = 0;
//	char    *ptr = string;

//	/* This implementation only works for decimal numbers. */
//	if (radix != 10)
//	{
//		*ptr = 0;
//		return string;
//	}

//	if (!value)
//	{
//		*ptr++ = 0x30;
//		*ptr = 0;
//		return string;
//	}

//	/* if this is a negative value insert the minus sign. */
//	if (value < 0)
//	{
//		*ptr++ = '-';

//		/* Make the value positive. */
//		value *= -1;
//		
//	}

//	for (i = 10000; i > 0; i /= 10)
//	{
//		d = value / i;

//		if (d || flag)
//		{
//			*ptr++ = (char)(d + 0x30);
//			value -= (d * i);
//			flag = 1;
//		}
//	}

//	/* Null terminate the string. */
//	*ptr = 0;

//	return string;

//} /* NCL_Itoa */







