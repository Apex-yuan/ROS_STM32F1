/**
  ******************************************************************************
  * @file    readme.txt
  * @author  ��Ԫ
  * @version V1.0
  * @date    2016-4-4
  * @brief   ���ڼ�¼�����ĵ��ı����־
  ******************************************************************************
  * @attention
  *
  * оƬ�ͺ�: STM32F103VET6 
  *
  ******************************************************************************
  */ 
  2016.4.12  xiaoyuan
  �ù����ļ��ṹ��
  USER-------stm32f10x.c,stm32f10x.h���жϷ���������stm32f10x_conf.h��Ƭ����������ͷ�ļ�����main.c����������
  HARDWARE---����û���д��Ӳ����Դ����
  FWLIB------STM32F10x_StdPeriph_Lib_V3.5.0 ��V3.5�̼��⣩
  CMSIS------system_stm32f10x.c,core_cm3.c  ���������躯���ļ�����stm32f10x.h ������Ĵ���������ж��������壩  
             system_stm32f10x.h,core_cm3.h
  STARTCODE--��������
  DOC--------����˵���ĵ�
  
  һЩ��Ҫ�ļ��Ľ��ܣ�
  core_cm3.c��core_cm3.h��λ��CMSIS��׼�ĺ����豸�������CM3��ͨ���ļ���ΪоƬ�����ṩһ������CM3�ں˵Ľӿ�
  system_stm32f10x.c��system_stm32f10x.h ����ϵͳ��ʱ�Ӻ�����ʱ�ӡ�
  stm32f10x.h ������STM32�мĴ�����ַ�ͽṹ�����Ͷ��壬��ʹ�õ�STM32�̼���ĵط���Ҫ�������ͷ�ļ���
  �����ļ������κδ��������ϵ縴λ֮���������е�һ�λ������û������ΪC���Ե����н���һ�����ʵ����л�����
           ��1����ʼ����ջָ��SP
		   ��2����ʼ�����������ָ��PC
		   ��3�����ö�ջ�Ĵ�С
		   ��4�������쳣���������ڵ�ַ
		   ��5�������ⲿSRAM��Ϊ���ݴ洢�� �����û����ã�
		   ��6������C��ķ�֧���__main��������������main������
		   
�������͵�һЩ���壬������stdint.h�ļ��У�
      /* exact-width signed integer types */
typedef   signed          char int8_t;   //�ַ��ͱ���
typedef   signed short     int int16_t;  //�з��Ŷ�����
typedef   signed           int int32_t;  //�з��ų�����
typedef   signed       __INT64 int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;   //�޷����ַ��� 
typedef unsigned short     int uint16_t;  //�޷��Ŷ�����
typedef unsigned           int uint32_t;  //�޷��ų�����
typedef unsigned       __INT64 uint64_t;

2019/3/19


  