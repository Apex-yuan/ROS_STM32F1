/**
  ******************************************************************************
  * @file    readme.txt
  * @author  王元
  * @version V1.0
  * @date    2016-4-4
  * @brief   用于记录工程文档改变的日志
  ******************************************************************************
  * @attention
  *
  * 芯片型号: STM32F103VET6 
  *
  ******************************************************************************
  */ 
  2016.4.12  xiaoyuan
  该工程文件结构：
  USER-------stm32f10x.c,stm32f10x.h（中断服务函数）；stm32f10x_conf.h（片上外设配置头文件）；main.c（主函数）
  HARDWARE---存放用户编写的硬件资源程序
  FWLIB------STM32F10x_StdPeriph_Lib_V3.5.0 （V3.5固件库）
  CMSIS------system_stm32f10x.c,core_cm3.c  （核内外设函数文件）；stm32f10x.h （外设寄存器定义和中断向量表定义）  
             system_stm32f10x.h,core_cm3.h
  STARTCODE--启动代码
  DOC--------工程说明文档
  
  一些重要文件的介绍：
  core_cm3.c，core_cm3.h是位于CMSIS标准的核内设备函数层的CM3核通用文件，为芯片外设提供一个进入CM3内核的接口
  system_stm32f10x.c，system_stm32f10x.h 设置系统的时钟和总线时钟。
  stm32f10x.h 包含了STM32中寄存器地址和结构体类型定义，在使用到STM32固件库的地方都要包含这个头文件。
  启动文件：是任何处理器在上电复位之后最先运行的一段汇编程序（用汇编语言为C语言的运行建立一个合适的运行环境）
           （1）初始化堆栈指针SP
		   （2）初始化程序计数器指针PC
		   （3）设置堆栈的大小
		   （4）设置异常向量表的入口地址
		   （5）配置外部SRAM作为数据存储器 （由用户配置）
		   （6）设置C库的分支入口__main（最终用来调用main函数）
		   
常用类型的一些定义，定义在stdint.h文件中：
      /* exact-width signed integer types */
typedef   signed          char int8_t;   //字符型变量
typedef   signed short     int int16_t;  //有符号短整形
typedef   signed           int int32_t;  //有符号长整形
typedef   signed       __INT64 int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;   //无符号字符型 
typedef unsigned short     int uint16_t;  //无符号短整形
typedef unsigned           int uint32_t;  //无符号长整形
typedef unsigned       __INT64 uint64_t;

2019/3/19


  