/**
  ******************************************************************************
  * @file    usart3.c
  * @author  xiaoyuan
  * @version V1.0
  * @date    2018-11-06
  * @brief   USART2模块初始化函数，
  *          提供了中断接收函数
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
  * @brief  初始化串口3
  * @param  bound：串口波特率，一般设置为115200
  * @retval None
  */
void usart3_init(uint32_t bound) 
{
  /*定义初始化结构体*/
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	/* 开启GPIOB端口时钟及USART3串口时钟 */
  RCC_APB1PeriphClockCmd(USART3_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(USART3_GPIO_CLK, ENABLE); 
 	
	/* 复位串口1 */
	USART_DeInit(USART3);  //复位串口1
	
	/* 配置USART3_TX（PB10）引脚GPIO状态 */
  GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(USART3_TX_PORT, &GPIO_InitStructure);   

  /* 配置USART3_RX（PB11）引脚GPIO状态 */
  GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(USART3_RX_PORT, &GPIO_InitStructure);  

  /* USART 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;  //波特率：一般设置为115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长：为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //停止位：一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;  //校验位：无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制:无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//工作模式：收发模式
  USART_Init(USART3, &USART_InitStructure);  //根据指定的参数初始化串口1
  
  /* 配置嵌套向量中断控制器NVIC */ 
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  //配置USART1为中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	 //根据指定的参数初始化NVIC寄存器
	
  /* 使能串口接收中断 */ 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  /* 使能串口溢出中断 */
  //USART_ITConfig (USART3, USART_IT_ORE, ENABLE );
  /* 串口3使能 */
  USART_Cmd(USART3, ENABLE);
}


//串口2,printf 函数
//确保一次发送数据不超过USART2_MAX_SEND_LEN字节
void usart3_printf(char* fmt,...)  
{
  char buffer[USART3_BUFFER_LEN+1];  
	uint16_t i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)buffer,fmt,ap);
	va_end(ap);
	i=strlen((const char*)buffer);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(USART3,buffer[j]); 
	} 
}



///*
// * 函数名：usart3_printf
// * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
// * 输入  ：-Data   要发送到串口的内容的指针
// *			   -...    其他参数
// * 输出  ：无
// * 返回  ：无 
// * 调用  ：外部调用
// *         典型应用USART2_printf("\r\n this is a demo \r\n" );
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

//	while ( * Data != 0 )     // 判断是否到达字符串结束符
//	{				                          
//		if ( * Data == 0x5c )  //'\'
//		{									  
//			switch ( *++Data )
//			{
//				case 'r':							          //回车符
//				USART_SendData(USART3, 0x0d);
//				Data ++;
//				break;

//				case 'n':							          //换行符
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
//				case 's':										  //字符串
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
//					//十进制
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
// * 函数名：itoa
// * 描述  ：将整形数据转换成字符串
// * 输入  ：-radix =10 表示10进制，其他结果为0
// *         -value 要转换的整形数
// *         -buf 转换后的字符串
// *         -radix = 10
// * 输出  ：无
// * 返回  ：无
// * 调用  ：被USART2_printf()调用
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







