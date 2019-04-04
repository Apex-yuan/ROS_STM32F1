/**
  ******************************************************************************
  * @file    usart1.c
  * @author  xiaoyuan
  * @version V2.0
  * @date    2018-1-28
  * @brief   USART模块初始化函数，提供了两种不同的方式支持printf函数，
  *          提供了中断接收函数
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "usart1.h"


/**
  * @brief  初始化串口1
  * @param  baud：串口波特率，一般设置为115200
  * @retval None
  */
void usart1_init(uint32_t bound) 
{
  /*定义初始化结构体*/
  GPIO_InitTypeDef   GPIO_InitStructure;
	USART_InitTypeDef  USART_InitStructure;
	#if EN_USART1_RX		
	NVIC_InitTypeDef   NVIC_InitStructure;
	#endif
	
	/* 开启GPIOA端口时钟及USART1串口时钟 */
	RCC_APB2PeriphClockCmd(USART1_CLK | USART1_GPIO_CLK |RCC_APB2Periph_AFIO, ENABLE);		//使能USART1，GPIOA和端口复用时钟
 	
	/* 复位串口1 */
	USART_DeInit(USART1);  //复位串口1
	
	/* 配置USART1_TX（PA9）引脚GPIO状态 */
  GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(USART1_TX_PORT, &GPIO_InitStructure);   

  /* 配置USART1_RX（PA10）引脚GPIO状态 */
  GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
  GPIO_Init(USART1_RX_PORT, &GPIO_InitStructure);  

  /* USART 初始化设置 */
	USART_InitStructure.USART_BaudRate = bound;  //波特率：一般设置为115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //字长：为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //停止位：一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;  //校验位：无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制:无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//工作模式：收发模式
  USART_Init(USART1, &USART_InitStructure);  //根据指定的参数初始化串口1

  #if EN_USART1_RX  
  /* 配置嵌套向量中断控制器NVIC */ 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  //配置USART1为中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	 //根据指定的参数初始化NVIC寄存器
	
  /* 使能串口接收中断 */ 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  #endif
  /* 串口1使能 */
  USART_Cmd(USART1, ENABLE);  //使能串口 
}

/* 选择不同的方式支持printf函数，要包含stdio.h头文件 */
#ifdef USE_MICROLIB  //使用microLib的方法支持printf函数 

/**
  * @brief  重新定向c库函数printf到串口1，重新定向后可使用printf函数打印信息
  * @param  ch：要发送的字节数据
  * @param  *f
  * @retval ch
  */
int fputc(int ch, FILE *f)
{
  /* 发送一个字节数据到串口 */
  USART_SendData(USART1, (uint8_t) ch);
  
  /* 等待发送完毕 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		

  return (ch);
}

/**
  * @brief  重新定向c库函数scanf到串口1，重新定向后可使用scanf、getchar函数
  * @param  *f
  * @retval 串口接收到的数据
  */
int fgetc(FILE *f)
{
  /* 等待串口输入数据 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

  return (int)USART_ReceiveData(USART1);
}

#else  //不使用microlib就可以支持printf函数  

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;

/* 定义_sys_exit()以避免使用半主机模式 */   
void _sys_exit(int x) 
{ 
	x = x; 
}

/**
  * @brief  重新定向c库函数printf到串口1，重新定向后可使用printf函数打印信息
  * @param  ch：要发送的字节数据
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

#if EN_USART1_RX   //如果使能了接收

///**
//  * @brief  重新定向c库函数printf到串口1，重新定向后可使用printf函数打印信息
//  * @param  ch：要发送的字节数据
//  * @param  *f
//  * @retval ch
//  */ 
//void USART1_IRQHandler(void)                	//串口1中断服务程序
//{
//	uint8_t rec;
//	
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d //0x0a结尾)
//	{
//		rec = USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据

//  } 
//} 

#endif

