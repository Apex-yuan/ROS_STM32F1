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

#define BUFFER_SIZE 4
#define MAX_PACK_SIZE 80

static char cmdBuffer[BUFFER_SIZE][MAX_PACK_SIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen  = 0;
int serial_count = 0;


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
    
    usart3_putBuff(cmdf,sizeof(cmdf));
    usart3_putBuff(wareaddr,waresize);
    usart3_putBuff(cmdr,sizeof(cmdr));
}
 
/**
  * @brief  协议解包处理
  * @param  none
  * @retval none
  */ 
void protocol_process(void)
{
  if(buflen)
  {
    switch(cmdBuffer[bufindr][1])
    {
      case '0':
        g_fBTSpeedSet = 0;
        g_fBTDirectionSet = 0; 
        break;
      case '1':
        g_fBTSpeedSet = 1.3;
        g_fBTDirectionSet = 0;
        break;
      case '2':   
        g_fBTSpeedSet = (-1.3);
        g_fBTDirectionSet = 0;
        break;
      case '3':
        g_fBTSpeedSet = 0;
        g_fBTDirectionSet = 1.0;
        break;
      case '4':
        g_fBTSpeedSet = 0;
        g_fBTDirectionSet = (-1.0);
        break;
      default:
        g_fBTSpeedSet = 0;
        g_fBTDirectionSet = 0;
      break;
    }
    buflen -= 1;
    bufindr = (bufindr + 1) % BUFFER_SIZE;
  }
}

/**
  * @brief  USART3串口中断服务函数
  * @param  none
  * @retval none
  */ 
void usart3_irq(void)
{
	uint8_t rec;
  
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		rec = USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
    if(rec == '$')
    {
      g_bStartBitFlag = 1;
      serial_count = 0;
    }
    if(g_bStartBitFlag == 1)
    {
      cmdBuffer[bufindw][serial_count++] = rec;
    }
    if(g_bStartBitFlag == 1 && rec == '#')
    {
      g_bStartBitFlag = 0;
      bufindw = (bufindw + 1) % BUFFER_SIZE;
      buflen += 1;
    }
    if(serial_count >= 80)
    {
      g_bStartBitFlag = 0;
      serial_count = 0;
    }
  } 
} 