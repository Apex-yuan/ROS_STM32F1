#include "protocol.h"
#include "speed_control.h"
#include "direction_control.h"
#include <string.h>

float g_fware[8];

bool g_bStartBitFlag = 0;
bool g_bNewLineFlag = 0;
uint32_t g_nCmdCount = 0;
uint8_t g_nCmdBuf[80] = {0};
uint8_t g_nProtocolBuf[80] = {0};
char returntemp[] = "$0,0,0,0,0,0,0,0,0,0,0,0cm,8.2V#";

void usart3_send_char(uint8_t c)
{
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET); //循环发送,直到发送完毕
	USART_SendData(USART3,c);
}

/*!
 *  @brief      发送指定len个字节长度数组 （包括 NULL 也会发送）
 *  @param      UARTn_e       模块号（UART0~UART5）
 *  @param      buff        数组地址
 *  @param      len         发送数组的长度
 *  @since      v5.0
 *  Sample usage:       uart_putbuff (UART3,"1234567", 3); //实际发送了3个字节'1','2','3'
 */
void usart3_putbuff (uint8_t *buff, uint32_t len)
{
    while(len--)
    {
        usart3_send_char(*buff);
        buff++;
    }
}

void usart3_niming_report(uint8_t fun,uint8_t*data,uint8_t len)
{
	uint8_t send_buf[32];
	uint8_t i;
	if(len>28)return;	//最多28字节数据
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0XAA;	//帧头
  send_buf[1]=0X00; //发送设备
  send_buf[2]=0xAF;
	send_buf[3]=fun;	//功能字
	send_buf[4]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[5+i]=data[i];			//复制数据
	for(i=0;i<len+5;i++)send_buf[len+4]+=send_buf[i];	//计算校验和
	for(i=0;i<len+5;i++)usart3_send_char(send_buf[i]);	//发送数据到串口1
}


void Send_Data(int16_t *Gyro,int16_t *Accel)
{
  unsigned char i=0;
  unsigned char _cnt=0,sum = 0;
  // unsigned int _temp;
  uint8_t data_to_send[50];
  data_to_send[_cnt++]=0xAA;
  data_to_send[_cnt++]=0x01;
  data_to_send[_cnt++]=0xAF;
  data_to_send[_cnt++]=0x02;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=BYTE1(Accel[0]);
  data_to_send[_cnt++]=BYTE0(Accel[0]);
  data_to_send[_cnt++]=BYTE1(Accel[1]);
  data_to_send[_cnt++]=BYTE0(Accel[1]);
  data_to_send[_cnt++]=BYTE1(Accel[2]);
  data_to_send[_cnt++]=BYTE0(Accel[2]);
  data_to_send[_cnt++]=BYTE1(Gyro[0]);
  data_to_send[_cnt++]=BYTE0(Gyro[0]);
  data_to_send[_cnt++]=BYTE1(Gyro[1]);
  data_to_send[_cnt++]=BYTE0(Gyro[1]);
  data_to_send[_cnt++]=BYTE1(Gyro[2]);
  data_to_send[_cnt++]=BYTE0(Gyro[2]);
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=0;
  data_to_send[_cnt++]=0;
  data_to_send[4] = _cnt-5;
  //和校验
  for (i=0; i<_cnt; i++)
  sum+= data_to_send[i];
  data_to_send[_cnt++]=sum;
  //串口发送数据
  for (i=0; i<_cnt; i++)
  usart3_send_char(data_to_send[i]);
}


/********************************************************************************************************
* Function name:           vcan_sendware
* Author/Corporation:      vcan
* Create date:             2015-07-05
* input parameters:        none
* output parameters:       none
* Return value:            none   
* Abstract Description:   发送AD值到虚拟示波器
*------------------------Revision History----------------------------------------------------------------
*  NO    Version      Date       Revised By    Description
*  1      V1.1      2016.6.6      xiaoyuan     规范代码风格
*********************************************************************************************************/
void vcan_sendware(uint8_t *wareaddr, uint32_t waresize)
{
    #define CMD_WARE     3   //3代表虚拟示波器,1摄像头，2CCD
  
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //yy_摄像头串口调试 使用的命令
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //yy_摄像头串口调试 使用的命令
    
    usart3_putbuff(cmdf,sizeof(cmdf));
    usart3_putbuff(wareaddr,waresize);
    usart3_putbuff(cmdr,sizeof(cmdr));
}



/**
  * @brief  USART3串口中断服务函数
  * @param  none
  * @retval none
  */ 
void USART3_IRQHandler(void) 
{
	uint8_t rec;
	
  if(USART_GetITStatus(USART3,USART_IT_ORE) == SET)
  {
    USART_ClearFlag(USART3,USART_IT_ORE);
    USART_ReceiveData(USART3); //由软件序列清除中断标志位(先读USART_SR，然后读USART_DR)	
  }
  
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		rec = USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
    if(rec == '$')
    {
      g_bStartBitFlag = 1;
      g_nCmdCount = 0;
    }
    if(g_bStartBitFlag == 1)
    {
      g_nCmdBuf[g_nCmdCount++] = rec;
    }
    if(g_bStartBitFlag == 1 && rec == '#')
    {
      g_bNewLineFlag = 1;
      g_bStartBitFlag = 0;
      //g_nCmdCount = 0;
    }
    if(g_nCmdCount >= 80)
    {
      g_bNewLineFlag = 0;
      g_bStartBitFlag = 0;
      g_nCmdCount = 0;
    }
  } 
} 

void ProtocolCpyData(void)
{
	memcpy(g_nProtocolBuf, g_nCmdBuf, g_nCmdCount+1);
	memset(g_nCmdBuf, 0x00, sizeof(g_nCmdBuf));
}

void usart3_send_str(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		usart3_send_char(*p);
		p++;
	}	
}

void Protocol(void)
{
  switch(g_nProtocolBuf[1])
  {
    case '0':
      g_fBTSpeedSet = 0;
      g_fDirectionSet = 0; 
      //usart3_send_str()
      usart3_send_str(returntemp);
      break;
    case '1':
      g_fBTSpeedSet = 1.3;
      g_fDirectionSet = 0;
      usart3_send_str(returntemp);
      break;
    case '2':   
      g_fBTSpeedSet = (-1.3);
      g_fDirectionSet = 0;
      usart3_send_str(returntemp);
      break;
    case '3':
      g_fBTSpeedSet = 0;
      g_fDirectionSet = 250;
      usart3_send_str(returntemp);
      break;
    case '4':
      g_fBTSpeedSet = 0;
      g_fDirectionSet = -250;
      usart3_send_str(returntemp);
      break;
    default:
      g_fBTSpeedSet = 0;
      g_fDirectionSet = 0;
    usart3_send_str(returntemp);
    break;
  }
  //newLineReceived = 0;  
	memset(g_nProtocolBuf, 0x00, sizeof(g_nProtocolBuf));  
}

