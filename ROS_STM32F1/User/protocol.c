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
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET); //ѭ������,ֱ���������
	USART_SendData(USART3,c);
}

/*!
 *  @brief      ����ָ��len���ֽڳ������� ������ NULL Ҳ�ᷢ�ͣ�
 *  @param      UARTn_e       ģ��ţ�UART0~UART5��
 *  @param      buff        �����ַ
 *  @param      len         ��������ĳ���
 *  @since      v5.0
 *  Sample usage:       uart_putbuff (UART3,"1234567", 3); //ʵ�ʷ�����3���ֽ�'1','2','3'
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
	if(len>28)return;	//���28�ֽ�����
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0XAA;	//֡ͷ
  send_buf[1]=0X00; //�����豸
  send_buf[2]=0xAF;
	send_buf[3]=fun;	//������
	send_buf[4]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[5+i]=data[i];			//��������
	for(i=0;i<len+5;i++)send_buf[len+4]+=send_buf[i];	//����У���
	for(i=0;i<len+5;i++)usart3_send_char(send_buf[i]);	//�������ݵ�����1
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
  //��У��
  for (i=0; i<_cnt; i++)
  sum+= data_to_send[i];
  data_to_send[_cnt++]=sum;
  //���ڷ�������
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
* Abstract Description:   ����ADֵ������ʾ����
*------------------------Revision History----------------------------------------------------------------
*  NO    Version      Date       Revised By    Description
*  1      V1.1      2016.6.6      xiaoyuan     �淶������
*********************************************************************************************************/
void vcan_sendware(uint8_t *wareaddr, uint32_t waresize)
{
    #define CMD_WARE     3   //3��������ʾ����,1����ͷ��2CCD
  
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //yy_����ͷ���ڵ��� ʹ�õ�����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //yy_����ͷ���ڵ��� ʹ�õ�����
    
    usart3_putbuff(cmdf,sizeof(cmdf));
    usart3_putbuff(wareaddr,waresize);
    usart3_putbuff(cmdr,sizeof(cmdr));
}



/**
  * @brief  USART3�����жϷ�����
  * @param  none
  * @retval none
  */ 
void USART3_IRQHandler(void) 
{
	uint8_t rec;
	
  if(USART_GetITStatus(USART3,USART_IT_ORE) == SET)
  {
    USART_ClearFlag(USART3,USART_IT_ORE);
    USART_ReceiveData(USART3); //�������������жϱ�־λ(�ȶ�USART_SR��Ȼ���USART_DR)	
  }
  
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{
		rec = USART_ReceiveData(USART3);//(USART1->DR);	//��ȡ���յ�������
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

