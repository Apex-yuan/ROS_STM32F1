/**
  ******************************************************************************
  * @file    usart.h
  * @author  xiaoyuan
  * @version V2.0
  * @date    2018-1-28
  * @brief   提供了USART模块函数的声明
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"
/* 要用到printf函数，因此要包含C语言的标准输入输出文件 */
#include "stdio.h"  //在此处包含stdio.h文件因此，在外部文件中只需包含usart.h便可以直接使用printf、scanf等函数

#define USART1_CLK        RCC_APB2Periph_USART1
#define USART1_GPIO_CLK   RCC_APB2Periph_GPIOA

#define USART1_TX_PORT    GPIOA
#define USART1_TX_PIN     GPIO_Pin_9
#define USART1_RX_PORT    GPIOA
#define USART1_RX_PIN     GPIO_Pin_10


#define USE_MICROLIB      1  //(1)使用微库/（0）不使用微库 ，选择使用微库需要在Option->Target选项栏中勾选上USE MicroLIB
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收



void usart1_init(uint32_t baud);


#endif //__USART_H
